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

#include "CDCSerial.h"
#include "string.h"

size_t min(size_t a,size_t b) {
  return a > b ? b : a;
}

void CDC_SendData(void) {
    uint32_t size = s_sendSize;
    int error;

    error = USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, size);

    if (error != kStatus_USB_Success)
    {
      /* Failure to send Data Handling code here */
      return;
    }
    s_sendSize   = 0;
}

void CDC_FlushBuffer(void) {
  uint32_t i;

  if(UsbSerial.transmit_buffer.available() > 0){
    s_sendSize = UsbSerial.transmit_buffer.available();

    for (i = 0; i < s_sendSize; i++)
    {
      s_currSendBuf[i] = UsbSerial.transmit_buffer.buffer[i];
    }

    CDC_SendData();

    UsbSerial.transmit_buffer.index_write = 0;
  }
}

void CDC_FillBuffer(uint32_t free_size) {
  if(free_size > 0){
    if(0 != s_recvSize){
      uint8_t copy_size  = min(free_size,s_recvSize);

      // PRINTF("Fill %d bytes\r\n",copy_size);

      if(UsbSerial.receive_buffer.index_write + copy_size < UsbSerial.receive_buffer.size()){
        memcpy(UsbSerial.receive_buffer.buffer + UsbSerial.receive_buffer.index_write,s_currRecvBuf,copy_size);
      }else{
        memcpy(UsbSerial.receive_buffer.buffer + UsbSerial.receive_buffer.index_write,
              s_currRecvBuf,
              UsbSerial.receive_buffer.size() - UsbSerial.receive_buffer.index_write);

        memcpy(UsbSerial.receive_buffer.buffer,
              s_currRecvBuf + UsbSerial.receive_buffer.size() - UsbSerial.receive_buffer.index_write,
              copy_size - UsbSerial.receive_buffer.size() + UsbSerial.receive_buffer.index_write);

      }
      s_recvSize = 0;
      UsbSerial.receive_buffer.index_write = (UsbSerial.receive_buffer.index_write + copy_size)
                          & UsbSerial.receive_buffer.buffer_mask;
      USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, 0);// continute to receive
    }
  }
}


CDCSerial UsbSerial;

// Function that can be weakly referenced by serialEventRun to prevent
// pulling in this file if it's not otherwise used.
bool SerialCDC_available() {
  return UsbSerial.available();
}

void fs_usb_loop(void) {
    UsbSerial.host_connected = s_cdcVcom.attach;
    CDC_FillBuffer(UsbSerial.receive_buffer.free());
}
