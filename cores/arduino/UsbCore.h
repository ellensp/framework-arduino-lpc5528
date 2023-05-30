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

#ifndef _USBCORE_H_
#define _USBCORE_H_

#pragma once

#include <stdint.h>

// Not used anymore? If anyone uses this, please let us know so that this may be
// moved to the proper place, settings.h.
//#define USB_METHODS_INLINE

/* shield pins. First parameter - SS pin, second parameter - INT pin */

/* Common setup data constant combinations  */
#define bmREQ_GET_DESCR     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     //get descriptor request type
#define bmREQ_SET           USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     //set request type for all but 'set feature' and 'set interface'
#define bmREQ_CL_GET_INTF   USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE     //get interface request type

// D7           data transfer direction (0 - host-to-device, 1 - device-to-host)
// D6-5         Type (0- standard, 1 - class, 2 - vendor, 3 - reserved)
// D4-0         Recipient (0 - device, 1 - interface, 2 - endpoint, 3 - other, 4..31 - reserved)

// USB Device Classes
#define USB_CLASS_USE_CLASS_INFO        0x00    // Use Class Info in the Interface Descriptors
#define USB_CLASS_AUDIO                 0x01    // Audio
#define USB_CLASS_COM_AND_CDC_CTRL      0x02    // Communications and CDC Control
#define USB_CLASS_HID                   0x03    // HID
#define USB_CLASS_PHYSICAL              0x05    // Physical
#define USB_CLASS_IMAGE                 0x06    // Image
#define USB_CLASS_PRINTER               0x07    // Printer
#define USB_CLASS_MASS_STORAGE          0x08    // Mass Storage
#define USB_CLASS_HUB                   0x09    // Hub
#define USB_CLASS_CDC_DATA              0x0A    // CDC-Data
#define USB_CLASS_SMART_CARD            0x0B    // Smart-Card
#define USB_CLASS_CONTENT_SECURITY      0x0D    // Content Security
#define USB_CLASS_VIDEO                 0x0E    // Video
#define USB_CLASS_PERSONAL_HEALTH       0x0F    // Personal Healthcare
#define USB_CLASS_DIAGNOSTIC_DEVICE     0xDC    // Diagnostic Device
#define USB_CLASS_WIRELESS_CTRL         0xE0    // Wireless Controller
#define USB_CLASS_MISC                  0xEF    // Miscellaneous
#define USB_CLASS_APP_SPECIFIC          0xFE    // Application Specific
#define USB_CLASS_VENDOR_SPECIFIC       0xFF    // Vendor Specific

// Additional Error Codes
#define USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED       0xD1
#define USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE     0xD2
#define USB_ERROR_UNABLE_TO_REGISTER_DEVICE_CLASS       0xD3
#define USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL          0xD4
#define USB_ERROR_HUB_ADDRESS_OVERFLOW                  0xD5
#define USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL             0xD6
#define USB_ERROR_EPINFO_IS_NULL                        0xD7
#define USB_ERROR_INVALID_ARGUMENT                      0xD8
#define USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE         0xD9
#define USB_ERROR_INVALID_MAX_PKT_SIZE                  0xDA
#define USB_ERROR_EP_NOT_FOUND_IN_TBL                   0xDB
#define USB_ERROR_CONFIG_REQUIRES_ADDITIONAL_RESET      0xE0
#define USB_ERROR_FailGetDevDescr                       0xE1
#define USB_ERROR_FailSetDevTblEntry                    0xE2
#define USB_ERROR_FailGetConfDescr                      0xE3
#define USB_ERROR_TRANSFER_TIMEOUT                      0xFF

#define USB_XFER_TIMEOUT        5000    // (5000) USB transfer timeout in milliseconds, per section 9.2.6.1 of USB 2.0 spec
//#define USB_NAK_LIMIT         32000   // NAK limit for a transfer. 0 means NAKs are not counted
#define USB_RETRY_LIMIT         3       // 3 retry limit for a transfer
#define USB_SETTLE_DELAY        200     // settle delay in milliseconds

#define USB_NUMDEVICES          16      //number of USB devices
//#define HUB_MAX_HUBS          7       // maximum number of hubs that can be attached to the host controller
#define HUB_PORT_RESET_DELAY    20      // hub port reset delay 10 ms recommended, can be up to 20 ms

/* USB state machine states */
#define USB_STATE_MASK                                      0xF0

#define USB_STATE_DETACHED                                  0x10
#define USB_DETACHED_SUBSTATE_INITIALIZE                    0x11
#define USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE               0x12
#define USB_DETACHED_SUBSTATE_ILLEGAL                       0x13
#define USB_ATTACHED_SUBSTATE_SETTLE                        0x20
#define USB_ATTACHED_SUBSTATE_RESET_DEVICE                  0x30
#define USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE           0x40
#define USB_ATTACHED_SUBSTATE_WAIT_SOF                      0x50
#define USB_ATTACHED_SUBSTATE_WAIT_RESET                    0x51
#define USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE    0x60
#define USB_STATE_ADDRESSING                                0x70
#define USB_STATE_CONFIGURING                               0x80
// #define USB_STATE_RUNNING                                   0x90
// #define USB_STATE_ERROR                                     0xA0

class USBDeviceConfig {
public:

  virtual uint8_t Init(uint8_t parent __attribute__((unused)), uint8_t port __attribute__((unused)), bool lowspeed __attribute__((unused))) {
    return 0;
  }

  virtual uint8_t ConfigureDevice(uint8_t parent __attribute__((unused)), uint8_t port __attribute__((unused)), bool lowspeed __attribute__((unused))) {
    return 0;
  }

  virtual uint8_t Release() {
    return 0;
  }

  virtual uint8_t Poll() {
    return 0;
  }

  virtual uint8_t GetAddress() {
    return 0;
  }

  virtual void ResetHubPort(uint8_t port __attribute__((unused))) {
    return;
  } // Note used for hubs only!

  virtual bool VIDPIDOK(uint16_t vid __attribute__((unused)), uint16_t pid __attribute__((unused))) {
    return false;
  }

  virtual bool DEVCLASSOK(uint8_t klass __attribute__((unused))) {
    return false;
  }

  virtual bool DEVSUBCLASSOK(uint8_t subklass __attribute__((unused))) {
    return true;
  }

};

/* USB Setup Packet Structure   */
typedef struct {

  union { // offset   description
    uint8_t bmRequestType; //   0      Bit-map of request type

    struct {
      uint8_t recipient : 5; //          Recipient of the request
      uint8_t type : 2; //          Type of request
      uint8_t direction : 1; //          Direction of data X-fer
    } __attribute__((packed));
  } ReqType_u;
  uint8_t bRequest; //   1      Request

  union {
    uint16_t wValue; //   2      Depends on bRequest

    struct {
      uint8_t wValueLo;
      uint8_t wValueHi;
    } __attribute__((packed));
  } wVal_u;
  uint16_t wIndex; //   4      Depends on bRequest
  uint16_t wLength; //   6      Depends on bRequest
} __attribute__((packed)) SETUP_PKT, *PSETUP_PKT;



// Base class for incoming data parser

class USBReadParser {
public:
  virtual void Parse(const uint16_t len, const uint8_t *pbuf, const uint16_t &offset) = 0;
};

#if 0 //defined(USB_METHODS_INLINE)
//get device descriptor

inline uint8_t USB::getDevDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t *dataptr) {
  return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00, USB_DESCRIPTOR_DEVICE, 0x0000, nbytes, dataptr));
}
//get configuration descriptor

inline uint8_t USB::getConfDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t conf, uint8_t *dataptr) {
  return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, nbytes, dataptr));
}
//get string descriptor

inline uint8_t USB::getStrDescr(uint8_t addr, uint8_t ep, uint16_t nuint8_ts, uint8_t index, uint16_t langid, uint8_t *dataptr) {
  return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, langid, nuint8_ts, dataptr));
}
//set address

inline uint8_t USB::setAddr(uint8_t oldaddr, uint8_t ep, uint8_t newaddr) {
  return ( ctrlReq(oldaddr, ep, bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00, 0x0000, 0x0000, nullptr));
}
//set configuration

inline uint8_t USB::setConf(uint8_t addr, uint8_t ep, uint8_t conf_value) {
  return ( ctrlReq(addr, ep, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000, 0x0000, nullptr));
}

#endif // defined(USB_METHODS_INLINE)

#endif //_USBCORE_H_
