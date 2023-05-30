/*
BSD 3-Clause License

Copyright (c) 2021-2022 WPI Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "printhex.h"
#include "masstorage1.h"
#include "message.h"
#include "stdint.h"
#include "wiring_time.h"
#include "usb_msc.h"

#include "usb_host_msd.h"
#include "usb_host.h"
#include "fsl_debug_console.h"

#include <usb_msc.h>

extern "C"{
  #include <host_msd_fatfs.h>
  #include <usb_host_hci.h>
  #include <usb_host_ip3516hs.h>
  #include <usb_host_devices.h>
}

static volatile uint8_t ufiIng;
static volatile usb_status_t ufiStatus;

static void USB_HostMsdUfiCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status) {
    ufiIng = 0;
    ufiStatus = status;
}

extern usb_host_class_handle g_UsbFatfsClassHandle;
extern usb_host_device_instance_t* g_HostDeviceInstance;

#define USB_NAK_MAX_POWER               15              //NAK binary order maximum value
#define USB_NAK_DEFAULT                 14              //default 32K-1 NAKs before giving up
#define USB_NAK_NOWAIT                  1               //Single NAK stops transfer
#define USB_NAK_NONAK                   0               //Do not count NAKs, stop retrying after USB Timeout

const uint8_t BulkOnly::epDataInIndex = 1;
const uint8_t BulkOnly::epDataOutIndex = 2;
const uint8_t BulkOnly::epInterruptInIndex = 3;

////////////////////////////////////////////////////////////////////////////////
// Interface code
////////////////////////////////////////////////////////////////////////////////

/**
 * Get the capacity of the media
 *
 * @param lun Logical Unit Number
 * @return media capacity
 */
uint32_t BulkOnly::GetCapacity(uint8_t lun) {
  if(lun > bMaxLUN) return 0;
  return CurrentCapacity[lun];
}

/**
 * Get the sector (block) size used on the media
 *
 * @param lun Logical Unit Number
 * @return media sector size
 */
uint16_t BulkOnly::GetSectorSize(uint8_t lun) {
  if(lun > bMaxLUN) return 0;
  return CurrentSectorSize[lun];
}

/**
 * Test if LUN is ready for use
 *
 * @param lun Logical Unit Number
 * @return true if LUN is ready for use
 */
bool BulkOnly::LUNIsGood(uint8_t lun) {
  if(lun > bMaxLUN) return false;
  return LUNOk[lun];
}

/**
 * Test if LUN is write protected
 *
 * @param lun Logical Unit Number
 * @return cached status of write protect switch
 */
bool BulkOnly::WriteProtected(uint8_t lun) {
   if(lun > bMaxLUN) return false;
   return WriteOk[lun];
}

/**
 * Wrap and execute a SCSI CDB with length of 6
 *
 * @param cdb CDB to execute
 * @param buf_size Size of expected transaction
 * @param buf Buffer
 * @param dir MASS_CMD_DIR_IN | MASS_CMD_DIR_OUT
 * @return
 */
uint8_t BulkOnly::SCSITransaction6(CDB6_t *cdb, uint16_t buf_size, void *buf, uint8_t dir) {
  CommandBlockWrapper cbw = CommandBlockWrapper(++dCBWTag, (uint32_t)buf_size, cdb, dir);
  return (HandleSCSIError(Transaction(&cbw, buf_size, buf)));
}

/**
 * Wrap and execute a SCSI CDB with length of 10
 *
 * @param cdb CDB to execute
 * @param buf_size Size of expected transaction
 * @param buf Buffer
 * @param dir MASS_CMD_DIR_IN | MASS_CMD_DIR_OUT
 * @return
 */
uint8_t BulkOnly::SCSITransaction10(CDB10_t *cdb, uint16_t buf_size, void *buf, uint8_t dir) {
  CommandBlockWrapper cbw = CommandBlockWrapper(++dCBWTag, (uint32_t)buf_size, cdb, dir);
  return HandleSCSIError(Transaction(&cbw, buf_size, buf));
}

/**
 * Lock or Unlock the tray or door on device.
 * Caution: Some devices with buggy firmware will lock up.
 *
 * @param lun Logical Unit Number
 * @param lock 1 to lock, 0 to unlock
 * @return
 */
uint8_t BulkOnly::LockMedia(uint8_t lun, uint8_t lock) {
  Notify(PSTR("\r\nLockMedia\r\n"), 0x80);
  Notify(PSTR("---------\r\n"), 0x80);

  CDB6_t cdb = CDB6_t(SCSI_CMD_PREVENT_REMOVAL, lun, (uint8_t)0, lock);
  return SCSITransaction6(&cdb, (uint16_t)0, nullptr, (uint8_t)MASS_CMD_DIR_IN);
}

/**
 * Media control, for spindle motor and media tray or door.
 * This includes CDROM, TAPE and anything with a media loader.
 *
 * @param lun Logical Unit Number
 * @param ctl 0x00 Stop Motor, 0x01 Start Motor, 0x02 Eject Media, 0x03 Load Media
 * @return 0 on success
 */
uint8_t BulkOnly::MediaCTL(uint8_t lun, uint8_t ctl) {
  Notify(PSTR("\r\nMediaCTL\r\n"), 0x80);
  Notify(PSTR("-----------------\r\n"), 0x80);

  if(lun > bMaxLUN)return MASS_ERR_INVALID_LUN;

  uint8_t rcode = MASS_ERR_UNIT_NOT_READY;
  if (bAddress) {
    CDB6_t cdb = CDB6_t(SCSI_CMD_START_STOP_UNIT, lun, ctl & 0x03, 0);
    rcode = SCSITransaction6(&cdb, (uint16_t)0, nullptr, (uint8_t)MASS_CMD_DIR_OUT);
  }
  return rcode;
}

/**
 * Read data from media
 *
 * @param lun Logical Unit Number
 * @param addr LBA address on media to read
 * @param bsize size of a block (we should probably use the cached size)
 * @param blocks how many blocks to read
 * @param buf memory that is able to hold the requested data
 * @return 0 on success
 */
uint8_t BulkOnly::Read(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, uint8_t *buf) {
  if(lun > bMaxLUN)return MASS_ERR_INVALID_LUN;
  if (!LUNOk[lun]) return MASS_ERR_NO_MEDIA;
  CDB10_t cdb = CDB10_t(SCSI_CMD_READ_10, lun, blocks, addr);

  return SCSITransaction10(&cdb, ((uint16_t)bsize * blocks), buf, (uint8_t)MASS_CMD_DIR_IN);
}

/**
 * Write data to media
 *
 * @param lun Logical Unit Number
 * @param addr LBA address on media to write
 * @param bsize size of a block (we should probably use the cached size)
 * @param blocks how many blocks to write
 * @param buf memory that contains the data to write
 * @return 0 on success
 */
uint8_t BulkOnly::Write(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, const uint8_t * buf) {
  if (!LUNOk[lun]) return MASS_ERR_NO_MEDIA;
  if (!WriteOk[lun]) return MASS_ERR_WRITE_PROTECTED;

  CDB10_t cdb = CDB10_t(SCSI_CMD_WRITE_10, lun, blocks, addr);
  return SCSITransaction10(&cdb, ((uint16_t)bsize * blocks), (void*)buf, (uint8_t)MASS_CMD_DIR_OUT);
}

// End of user functions, the remaining code below is driver internals.
// Only developer serviceable parts below!

////////////////////////////////////////////////////////////////////////////////
// Main driver code
////////////////////////////////////////////////////////////////////////////////

BulkOnly::BulkOnly(){
  ClearAllEP();
  dCBWTag = 0;
  usb1_host_app_init();
}

/**
 * USB_ERROR_CONFIG_REQUIRES_ADDITIONAL_RESET == success
 * We need to standardize either the rcode, or change the API to return values
 * so a signal that additional actions are required can be produced.
 * Some of these codes do exist already.
 *
 * TECHNICAL: We could do most of this code elsewhere, with the exception of checking the class instance.
 * Doing so would save some program memory when using multiple drivers.
 *
 * @param parent USB address of parent
 * @param port address of port on parent
 * @param lowspeed true if device is low speed
 * @return
 */
uint8_t BulkOnly::ConfigureDevice(uint8_t parent, uint8_t port, bool lowspeed) {
  return 0;
}

/**
 * @param parent (not used)
 * @param port (not used)
 * @param lowspeed true if device is low speed
 * @return 0 for success
 */
uint8_t BulkOnly::Init(uint8_t parent __attribute__((unused)), uint8_t port __attribute__((unused)), bool lowspeed __attribute__((unused))) {
    uint32_t cur_time = millis();

    while(millis() - cur_time < 1000) {
        hs_usb_loop();
    }

    if(bAddress){
      uint8_t rcode;
      // PRINTF("address:%d\r\n",bAddress); // dylan
      udisk.GetMaxLUN(&bMaxLUN);
      // PRINTF("max lun:%d\r\n",bMaxLUN); // dylan

      for (uint8_t lun = 0; lun <= bMaxLUN; lun++) {
        InquiryResponse response;
        rcode = Inquiry(lun, sizeof (InquiryResponse), (uint8_t*) & response);
        if (rcode) {
          // ErrorMessage<uint8_t> (PSTR("Inquiry"), rcode);// dylan
        }
        else {
          #if 1
          // PRINTF("LUN %i `", lun);
          uint8_t *buf = response.VendorID;
          for (int i = 0; i < 28; i++) PRINTF("%c", buf[i]);
          // PRINTF("'\r\nQualifier %1.1X ", response.PeripheralQualifier);
          // PRINTF("Device type %2.2X ", response.DeviceType);
          // PRINTF("RMB %1.1X ", response.Removable);
          // PRINTF("SSCS %1.1X ", response.SCCS);
          uint8_t sv = response.Version;
          // PRINTF("SCSI version %2.2X\r\nDevice conforms to ", sv);//dylan
          switch (sv) {
            case 0:
                    PRINTF("No specific");
                    break;
            case 1:
                    PRINTF("ANSI X3.131-1986 (ANSI 1)");
                    break;
            case 2:
                    PRINTF("ANSI X3.131-1994 (ANSI 2)");
                    break;
            case 3:
                    PRINTF("ANSI INCITS 301-1997 (SPC)");
                    break;
            case 4:
                    PRINTF("ANSI INCITS 351-2001 (SPC-2)");
                    break;
            case 5:
                    PRINTF("ANSI INCITS 408-2005 (SPC-4)");
                    break;
            case 6:
                    PRINTF("T10/1731-D (SPC-4)");
                    break;
            default: PRINTF("unknown");
          }
          PRINTF(" standards.\r\n");
          #endif

          uint8_t tries = 0xF0;
          while ((rcode = TestUnitReady(lun))) {
            tries++;
            if (!tries) break;
          }
          if (!rcode) {
            delay(1000);
            LUNOk[lun] = CheckLUN(lun);
            if (!LUNOk[lun]) LUNOk[lun] = CheckLUN(lun);
          }
          #if 1
          // PrintDescriptorsInfo(); //dylan
          #endif
        }
      }
    }else{
      NotifyStr("No Media\r\n",0x80);
      return MASS_ERR_NO_MEDIA;
    }

    bPollEnable = true;
    return MASS_ERR_SUCCESS;
}

/**
 * For driver use only.
 *
 * @param conf
 * @param iface
 * @param alt
 * @param proto
 * @param pep
 */
void BulkOnly::EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto __attribute__((unused)), const USB_FD_ENDPOINT_DESCRIPTOR * pep) {

}

/**
 * For driver use only.
 *
 * @return
 */
uint8_t BulkOnly::Release() {
    ClearAllEP();
    USB_HostDetachDevice(g_HostHandle, 0, 0);
    return MASS_ERR_SUCCESS;
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @return true if LUN is ready for use.
 */
bool BulkOnly::CheckLUN(uint8_t lun) {
    uint8_t rcode;
    static uint8_t buffer[8];

  rcode = ReadCapacity10(lun, (uint8_t*)buffer);
  if (rcode) {
    // NotifyStr("ReadCapacity failed\r\n",0x80); // dylan
    //printf(">>>>>>>>>>>>>>>>ReadCapacity returned %i\r\n", rcode);
    return false;
  }
  // ErrorMessage<uint8_t> (PSTR(">>>>>>>>>>>>>>>>CAPACITY OK ON LUN"), lun); // dylan
  // for (uint8_t i = 0; i < 8 ; i++)
  //         D_PrintHex<uint8_t> (buffer[i], 0x80);//dylan
  // Notify(PSTR("\r\n\r\n"), 0x80);//dylan

  // Only 512/1024/2048/4096 are valid values!
  uint32_t c = BMAKE32(buffer[4], buffer[5], buffer[6], buffer[7]);
  if (c != 0x0200UL && c != 0x0400UL && c != 0x0800UL && c != 0x1000UL) return false;

  CurrentSectorSize[lun] = (uint16_t)(c);
  CurrentCapacity[lun] = BMAKE32(buffer[0], buffer[1], buffer[2], buffer[3]) + 1;
  if (CurrentCapacity[lun] ==  0x01UL || CurrentCapacity[lun] == 0x00UL) {
    // Buggy firmware will report 0xFFFFFFFF or 0 for no media
    if (CurrentCapacity[lun])
      // ErrorMessage<uint8_t> (PSTR(">>>>>>>>>>>>>>>>BUGGY FIRMWARE. CAPACITY FAIL ON LUN"), lun);//dylan
    return false;
  }
  delay(20);
  Page3F(lun);
  return !TestUnitReady(lun);
}

/**
 * For driver use only.
 *
 * Scan for media change on all LUNs
 */
void BulkOnly::CheckMedia() {
  for (uint8_t lun = 0; lun <= bMaxLUN; lun++) {
    if (TestUnitReady(lun)) {
      LUNOk[lun] = false;
      continue;
    }
    if (!LUNOk[lun]) LUNOk[lun] = CheckLUN(lun);
  }
  qNextPollTime = (uint32_t)millis() + 2000;
  NotifyStr("Check media\r\n",0x80);
}

/**
 * For driver use only.
 *
 * @return
 */
uint8_t BulkOnly::Poll() {
  if (!bPollEnable) return 0;
  if ((int32_t)((uint32_t)millis() - qNextPollTime) >= 0L) CheckMedia();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// SCSI code
////////////////////////////////////////////////////////////////////////////////

/**
 * For driver use only.
 *
 * @param plun
 * @return
 */
uint8_t BulkOnly::GetMaxLUN(uint8_t *plun) {
      return USB_HostMsdGetMaxLun(g_UsbFatfsClassHandle,plun,NULL,NULL);
}

/**
 * For driver use only. Used during Driver Init
 *
 * @param lun Logical Unit Number
 * @param bsize
 * @param buf
 * @return
 */
uint8_t BulkOnly::Inquiry(uint8_t lun, uint16_t bsize, uint8_t *buf) {
  // Notify(PSTR("\r\nInquiry\r\n"), 0x80);
  // Notify(PSTR("---------\r\n"), 0x80);//by dylan

  CDB6_t cdb = CDB6_t(SCSI_CMD_INQUIRY, lun, 0UL, (uint8_t)bsize, 0);
  return SCSITransaction6(&cdb, bsize, buf, (uint8_t)MASS_CMD_DIR_IN);
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @return
 */
uint8_t BulkOnly::TestUnitReady(uint8_t lun) {
  //SetCurLUN(lun);
  if (!bAddress)
          return MASS_ERR_UNIT_NOT_READY;

  // Notify(PSTR("\r\nTestUnitReady\r\n"), 0x80);
  // Notify(PSTR("-----------------\r\n"), 0x80);//dylan

  CDB6_t cdb = CDB6_t(SCSI_CMD_TEST_UNIT_READY, lun, (uint8_t)0, 0);
  return SCSITransaction6(&cdb, 0, nullptr, (uint8_t)MASS_CMD_DIR_IN);

}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @param pc
 * @param page
 * @param subpage
 * @param len
 * @param pbuf
 * @return
 */
uint8_t BulkOnly::ModeSense6(uint8_t lun, uint8_t pc, uint8_t page, uint8_t subpage, uint8_t len, uint8_t * pbuf) {
  // Notify(PSTR("\r\rModeSense\r\n"), 0x80);
  // Notify(PSTR("------------\r\n"), 0x80); // dylan

  CDB6_t cdb = CDB6_t(SCSI_CMD_MODE_SENSE_6, lun, (uint32_t)((((pc << 6) | page) << 8) | subpage), len, 0);
  return SCSITransaction6(&cdb, len, pbuf, (uint8_t)MASS_CMD_DIR_IN);
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @param bsize
 * @param buf
 * @return
 */
uint8_t BulkOnly::ReadCapacity10(uint8_t lun, uint8_t *buf) {
  // Notify(PSTR("\r\nReadCapacity\r\n"), 0x80);
  // Notify(PSTR("---------------\r\n"), 0x80); // dylan

  CDB10_t cdb = CDB10_t(SCSI_CMD_READ_CAPACITY_10, lun);
  return SCSITransaction10(&cdb, 8, buf, (uint8_t)MASS_CMD_DIR_IN);
}

/**
 * For driver use only.
 *
 * Page 3F contains write protect status.
 *
 * @param lun Logical Unit Number to test.
 * @return Write protect switch status.
 */
uint8_t BulkOnly::Page3F(uint8_t lun) {
  static uint8_t buf[192];
  for (int i = 0; i < 192; i++) {
          buf[i] = 0x00;
  }
  WriteOk[lun] = true;
  #ifdef SKIP_WRITE_PROTECT
    return 0;
  #endif
  uint8_t rc = ModeSense6(lun, 0, 0x3F, 0, 192, buf);
  if (!rc) {
          WriteOk[lun] = ((buf[2] & 0x80) == 0);
          // Notify(PSTR("Mode Sense: "), 0x80);
          for (int i = 0; i < 4; i++) {
                  // D_PrintHex<uint8_t> (buf[i], 0x80);
                  // Notify(PSTR(" "), 0x80);
          }
          // Notify(PSTR("\r\n"), 0x80);//dylan
  }
  return rc;
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @param size
 * @param buf
 * @return
 */
uint8_t BulkOnly::RequestSense(uint8_t lun, uint16_t size, uint8_t *buf) {
  Notify(PSTR("\r\nRequestSense\r\n"), 0x80);
  Notify(PSTR("----------------\r\n"), 0x80);

  CDB6_t cdb = CDB6_t(SCSI_CMD_REQUEST_SENSE, lun, 0UL, (uint8_t)size, 0);
  CommandBlockWrapper cbw = CommandBlockWrapper(++dCBWTag, (uint32_t)size, &cdb, (uint8_t)MASS_CMD_DIR_IN);
  return Transaction(&cbw, size, buf);
}


////////////////////////////////////////////////////////////////////////////////
// USB code
////////////////////////////////////////////////////////////////////////////////

/**
 * For driver use only.
 *
 * @param index
 * @return
 */
uint8_t BulkOnly::ClearEpHalt(uint8_t index) {
  return 0;
}

/**
 * For driver use only.
 */
void BulkOnly::Reset() {
  USB_HostMsdMassStorageReset(g_UsbFatfsClassHandle,NULL,NULL);
}

/**
 * For driver use only.
 *
 * @return 0 if successful
 */
uint8_t BulkOnly::ResetRecovery() {
  Notify(PSTR("\r\nResetRecovery\r\n"), 0x80);
  Notify(PSTR("-----------------\r\n"), 0x80);

  delay(6);
  Reset();
  delay(6);
  ClearEpHalt(epDataInIndex);
  delay(6);
  bLastUsbError = ClearEpHalt(epDataOutIndex);
  delay(6);
  return bLastUsbError;
}

/**
 * For driver use only.
 *
 * Clear all EP data and clear all LUN status
 */
void BulkOnly::ClearAllEP() {
  for (uint8_t i = 0; i < MASS_MAX_ENDPOINTS; i++) {
    epInfo[i].epAddr = 0;
    epInfo[i].maxPktSize = (i) ? 0 : 8;
    epInfo[i].bmSndToggle = 0;
    epInfo[i].bmRcvToggle = 0;
    epInfo[i].bmNakPower = USB_NAK_DEFAULT;
  }

  for (uint8_t i = 0; i < MASS_MAX_SUPPORTED_LUN; i++) {
    LUNOk[i] = false;
    WriteOk[i] = false;
    CurrentCapacity[i] = 0UL;
    CurrentSectorSize[i] = 0;
  }

  bIface = 0;
  bNumEP = 1;
  bAddress = 0;
  qNextPollTime = 0;
  bPollEnable = false;
  bLastUsbError = 0;
  bMaxLUN = 0;
  bTheLUN = 0;
}

/**
 * For driver use only.
 *
 * @param pcbw
 * @param buf_size
 * @param buf
 * @param flags
 * @return
 */

uint8_t BulkOnly::Transaction(CommandBlockWrapper *pcbw, uint16_t buf_size, void *buf) {
  uint8_t cdb[10] = {0};
  int status;
  memcpy(cdb,pcbw->CBWCB,pcbw->bmCBWCBLength);

  status = USB_HostMsdCommand1(g_UsbFatfsClassHandle, (uint8_t*)buf, buf_size, USB_HostMsdUfiCallback, NULL,
                              pcbw->bmCBWFlags, cdb,pcbw->bmCBWCBLength);

  if(kStatus_USB_Success == status) {
      ufiIng = 1;
      while (ufiIng)
      {
          usb1_host_task(g_HostHandle);
      }
      return MASS_ERR_SUCCESS;
  } else {
    PRINTF("status:%d\r\n",status);
    return MASS_ERR_NO_MEDIA;
  }
}

/**
 * For driver use only.
 *
 * @param lun Logical Unit Number
 * @return
 */
uint8_t BulkOnly::SetCurLUN(uint8_t lun) {
  if (lun > bMaxLUN) return MASS_ERR_INVALID_LUN;
  bTheLUN = lun;
  return MASS_ERR_SUCCESS;
}

/**
 * For driver use only.
 *
 * @param status
 * @return
 */
uint8_t BulkOnly::HandleSCSIError(uint8_t status) {
  uint8_t ret = 0;

  switch (status) {
    case 0: return MASS_ERR_SUCCESS;

    case 2:
      ErrorMessage<uint8_t> (PSTR("Phase Error"), status);
      ErrorMessage<uint8_t> (PSTR("LUN"), bTheLUN);
      ResetRecovery();
      return MASS_ERR_GENERAL_SCSI_ERROR;

    case 1:
      ErrorMessage<uint8_t> (PSTR("SCSI Error"), status);
      ErrorMessage<uint8_t> (PSTR("LUN"), bTheLUN);
      RequestSenseResponce rsp;

      ret = RequestSense(bTheLUN, sizeof (RequestSenseResponce), (uint8_t*) & rsp);

      if (ret) return MASS_ERR_GENERAL_SCSI_ERROR;

      ErrorMessage<uint8_t> (PSTR("Response Code"), rsp.bResponseCode);
      if (rsp.bResponseCode & 0x80) {
        Notify(PSTR("Information field: "), 0x80);
        for (int i = 0; i < 4; i++) {
          D_PrintHex<uint8_t> (rsp.CmdSpecificInformation[i], 0x80);
          Notify(PSTR(" "), 0x80);
        }
        Notify(PSTR("\r\n"), 0x80);
      }
      ErrorMessage<uint8_t> (PSTR("Sense Key"), rsp.bmSenseKey);
      ErrorMessage<uint8_t> (PSTR("Add Sense Code"), rsp.bAdditionalSenseCode);
      ErrorMessage<uint8_t> (PSTR("Add Sense Qual"), rsp.bAdditionalSenseQualifier);
      // warning, this is not testing ASQ, only SK and ASC.
      switch (rsp.bmSenseKey) {
        case SCSI_S_UNIT_ATTENTION:
          switch (rsp.bAdditionalSenseCode) {
            case SCSI_ASC_MEDIA_CHANGED:
              return MASS_ERR_MEDIA_CHANGED;
            default:
              return MASS_ERR_UNIT_NOT_READY;
          }
        case SCSI_S_NOT_READY:
          switch (rsp.bAdditionalSenseCode) {
            case SCSI_ASC_MEDIUM_NOT_PRESENT:
              return MASS_ERR_NO_MEDIA;
            default:
              return MASS_ERR_UNIT_NOT_READY;
          }
        case SCSI_S_ILLEGAL_REQUEST:
          switch (rsp.bAdditionalSenseCode) {
            case SCSI_ASC_LBA_OUT_OF_RANGE:
              return MASS_ERR_BAD_LBA;
            default:
              return MASS_ERR_CMD_NOT_SUPPORTED;
          }
        default:
          return MASS_ERR_GENERAL_SCSI_ERROR;
      }

      // case 4: return MASS_ERR_UNIT_BUSY; // Busy means retry later.
      //    case 0x05/0x14: we stalled out
      //    case 0x15/0x16: we naked out.
    default:
      ErrorMessage<uint8_t> (PSTR("Gen SCSI Err"), status);
      ErrorMessage<uint8_t> (PSTR("LUN"), bTheLUN);
      return status;
  } // switch
}


////////////////////////////////////////////////////////////////////////////////
// Debugging code
////////////////////////////////////////////////////////////////////////////////

/**
 * @param ep_ptr
 */
void BulkOnly::PrintEndpointDescriptor(const USB_FD_ENDPOINT_DESCRIPTOR * ep_ptr) {
  Notify(PSTR("Endpoint descriptor:"), 0x80);
  Notify(PSTR("\r\nLength:\t\t"), 0x80);
  D_PrintHex<uint8_t> (ep_ptr->bLength, 0x80);
  Notify(PSTR("\r\nType:\t\t"), 0x80);
  D_PrintHex<uint8_t> (ep_ptr->bDescriptorType, 0x80);
  Notify(PSTR("\r\nAddress:\t"), 0x80);
  D_PrintHex<uint8_t> (ep_ptr->bEndpointAddress, 0x80);
  Notify(PSTR("\r\nAttributes:\t"), 0x80);
  D_PrintHex<uint8_t> (ep_ptr->bmAttributes, 0x80);
  Notify(PSTR("\r\nMaxPktSize:\t"), 0x80);
  D_PrintHex<uint16_t> (ep_ptr->wMaxPacketSize, 0x80);
  Notify(PSTR("\r\nPoll Intrv:\t"), 0x80);
  D_PrintHex<uint8_t> (ep_ptr->bInterval, 0x80);
  Notify(PSTR("\r\n"), 0x80);
}

/**
 * @param ep_ptr
 */
void BulkOnly::PrintDescriptorsInfo(void) {
  if(g_HostHandle != NULL){

    usb_descriptor_device_t* dev_descriptor = g_HostDeviceInstance->deviceDescriptor;
    usb_host_configuration_t* host_conf = &(g_HostDeviceInstance->configuration);
    usb_descriptor_configuration_t* conf_descriptor = host_conf->configurationDesc;
    usb_host_interface_t* host_interface = host_conf->interfaceList;
    usb_descriptor_interface_t* inf_descriptor = host_interface->interfaceDesc;
    usb_host_ep_t* endp_list = host_interface->epList;
    usb_descriptor_endpoint_t* endp_descriptor;

/* device descriptors */
    PRINTF("*******************************************************\r\n");
    PRINTF("Device descriptor:\r\n");
    PRINTF("Desp type:%d\r\n",dev_descriptor->bDescriptorType);
    PRINTF("Desp length:%d\r\n",dev_descriptor->bLength);
    PRINTF("Device Class:%d\r\n",dev_descriptor->bDeviceClass);
    PRINTF("Device Sub Class:%d\r\n",dev_descriptor->bDeviceSubClass);
    PRINTF("Device protocol:%d\r\n",dev_descriptor->bDeviceProtocol);
    PRINTF("Max pack size:%d\r\n",dev_descriptor->bMaxPacketSize0);
    PRINTF("Configuration num:%d\r\n",dev_descriptor->bNumConfigurations);
    PRINTF("Product ID:0x%x\r\n",(dev_descriptor->idProduct[1]<<8 | dev_descriptor->idProduct[0]));
    PRINTF("Vendor ID:0x%x\r\n",(dev_descriptor->idVendor[1]<<8 | dev_descriptor->idVendor[0]));
/* configuration descriptors */
    PRINTF("*******************************************************\r\n");
    PRINTF("Configuration descriptor:\r\n");
    PRINTF("Desp type:%d\r\n",conf_descriptor->bDescriptorType);
    PRINTF("Desp length:%d\r\n",conf_descriptor->bLength);
    PRINTF("Interface num:%d\r\n",conf_descriptor->bNumInterfaces);
    PRINTF("Total Desp length:%d\r\n",conf_descriptor->wTotalLength[1]<<8 | conf_descriptor->wTotalLength[0]);

/* Interface descriptors */      
    PRINTF("*******************************************************\r\n");
    PRINTF("Interface descriptor:\r\n");
    PRINTF("Desp type:%d\r\n",inf_descriptor->bDescriptorType);
    PRINTF("Desp length:%d\r\n",inf_descriptor->bLength);
    PRINTF("Interface index:%d\r\n",inf_descriptor->bInterfaceNumber);
    PRINTF("Interface protocol:%d\r\n",inf_descriptor->bInterfaceProtocol);
    PRINTF("Interface class:%d\r\n",inf_descriptor->bInterfaceClass);
    PRINTF("Interface sub class:%d\r\n",inf_descriptor->bInterfaceSubClass);
    PRINTF("Endpoint num:%d\r\n",inf_descriptor->bNumEndpoints);

/* Endpoints descriptors */
    PRINTF("*******************************************************\r\n");
    for(int i=0;i<inf_descriptor->bNumEndpoints;i++) {
      PRINTF("Endpoint%d descriptor:\r\n",i);
      endp_descriptor = endp_list[i].epDesc;
      PRINTF("Desp type:%d\r\n",endp_descriptor->bDescriptorType);
      PRINTF("Desp length:%d\r\n",endp_descriptor->bLength);
      PRINTF("Endpoint Attr:%d\r\n",endp_descriptor->bmAttributes);
      PRINTF("Endpoint max pack size:%d\r\n",endp_descriptor->wMaxPacketSize[1]<<8 | endp_descriptor->wMaxPacketSize[0]);
      PRINTF("Endpoint address:%d\r\n",endp_descriptor->bEndpointAddress);
      PRINTF("Endpoint internal:%d\r\n",endp_descriptor->bInterval);
      PRINTF("-------------------------------------\r\n");
    }
  }
}

BulkOnly udisk;
