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

#ifndef _CDCSERIAL_H_
#define _CDCSERIAL_H_

#include <stdarg.h>
#include <stdio.h>
#include <Stream.h>
#include <Arduino.h>

#include <usb_cdc.h>

#ifndef USBCDCTIMEOUT
  #define USBCDCTIMEOUT 6
#endif

size_t min(size_t a,size_t b);
void CDC_FlushBuffer(void);
void CDC_FillBuffer(uint32_t free_size);
void CDC_SendData(void);
void fs_usb_loop(void);

/**
 * Generic RingBuffer
 * T type of the buffer array
 * S size of the buffer (must be power of 2)
 */

template <typename T, std::size_t S> class RingBuffer {
public:
  RingBuffer() {index_read = index_write = 0;}

  std::size_t available() const {return mask(index_write - index_read);}
  std::size_t free() const {return size() - available();}
  bool empty() const {return index_read == index_write;}
  bool full() const {return next(index_write) == index_read;}
  void clear() {index_read = index_write = 0;}

  bool peek(T *const value) const {
    if (value == nullptr || empty()) return false;
    *value = buffer[index_read];
    return true;
  }

  [[gnu::always_inline, gnu::optimize("O3")]] inline std::size_t read(T* dst, std::size_t length) {
    length = min(length, available());
    const std::size_t length1 = min(length, buffer_size - index_read);
    memcpy(dst, (char*)buffer + index_read, length1);
    memcpy(dst + length1, (char*)buffer, length - length1);
    index_read = mask(index_read + length);
    return length;
  }

  [[gnu::always_inline, gnu::optimize("O3")]] inline std::size_t write(T* src, std::size_t length) {
    length = min(length, free());
    const std::size_t length1 = min(length, buffer_size - index_write);
    memcpy((char*)buffer + index_write, src, length1);
    memcpy((char*)buffer, src + length1, length - length1);
    index_write = mask(index_write + length);
    return length;
  }

  std::size_t read(T *const value) {
    if (value == nullptr || empty()) return 0;
    *value = buffer[index_read];
    index_read = next(index_read);
    return 1;
  }

  std::size_t write(const T value) {
    std::size_t next_head = next(index_write);
    if (next_head == index_read) return 0;     // buffer full
    buffer[index_write] = value;
    index_write = next_head;
    return 1;
  }

  constexpr std::size_t size() const {
    return buffer_size - 1;
  }

  static const std::size_t buffer_size = S;
  static const std::size_t buffer_mask = buffer_size - 1;
  T buffer[buffer_size];
  std::size_t index_write;
  std::size_t index_read;

private:
  inline std::size_t mask(std::size_t val) const {
    return val & buffer_mask;
  }

  inline std::size_t next(std::size_t val) const {
    return mask(val + 1);
  }

};

/**
 *  Serial Interface Class
 *  Data is injected directly into, and consumed from, the fifo buffers
 */

class CDCSerial: public Stream {
public:

  CDCSerial() : host_connected(false) { }
  virtual ~CDCSerial() { }

  operator bool() { return host_connected; }

  void begin(int32_t baud) { usb0_cdc_init();}

  void end() {};

  int16_t peek() {
    uint8_t value;
    return receive_buffer.peek(&value) ? value : -1;
  }

  int16_t read() {
    uint8_t value;
    uint32_t ret = receive_buffer.read(&value);
    CDC_FillBuffer(receive_buffer.free());
    return (ret ? value : -1);
  }

  size_t readBytes(char* dst, size_t length) {
    size_t buffered = receive_buffer.read((uint8_t*)dst, length);
    const uint32_t usb_rx_timeout = millis() + USBCDCTIMEOUT;
    while (buffered != length && (millis() < usb_rx_timeout)) {
      if (!host_connected) return 0;
      CDC_FillBuffer(receive_buffer.free());
      buffered += receive_buffer.read((uint8_t*)dst + buffered, length - buffered);
    }
    CDC_FillBuffer(receive_buffer.free());
    return buffered;
  }

  size_t write(char* src, size_t length) {
    size_t buffered = transmit_buffer.write((uint8_t*)src, length);
    const uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;
    while (buffered != length && (millis() < usb_tx_timeout)) {
      if (!host_connected) return 0;
      buffered += transmit_buffer.write((uint8_t*)src + buffered, length - buffered);
      CDC_FlushBuffer();
    }
    delay(1);//add 20211118
    CDC_FlushBuffer();
    return buffered;
  }

  size_t write(const uint8_t c) {
    if (!host_connected) return 0;          // Do not fill buffer when host disconnected
    const uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;
    while (transmit_buffer.write(c) == 0 && (millis() < usb_tx_timeout)) { // Block until there is free room in buffer
      if (!host_connected) return 0;        // Break infinite loop on host disconect
      CDC_FlushBuffer();
    }
    delay(1);//add 20211118
    CDC_FlushBuffer();
    return 1;
  }

  size_t available() {
    fs_usb_loop(); //add
    return receive_buffer.available();
  }

  void flush() {
    receive_buffer.clear();
    CDC_FillBuffer(receive_buffer.free());
  }

  uint8_t availableForWrite(void) {
    return min(transmit_buffer.free(), size_t(255));
  }

  void flushTX(void) {
    const uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;
    while (transmit_buffer.available() && host_connected && (millis() < usb_tx_timeout)) {
      CDC_FlushBuffer();
    }
  }

  size_t printf(const char *format, ...) {
    static uint8_t buffer[256];
    size_t buffered = 0;
    va_list vArgs;
    va_start(vArgs, format);
    volatile int length = vsnprintf((char *) buffer, 256, (char const *) format, vArgs);
    va_end(vArgs);

    if (length > 0 && length < 256) {
      uint32_t usb_tx_timeout = millis() + USBCDCTIMEOUT;

      buffered = transmit_buffer.write(buffer,length);
      while (buffered < (size_t)length && host_connected && (millis() < usb_tx_timeout)) {
        buffered += transmit_buffer.write((uint8_t*)buffer + buffered, length - buffered);
        CDC_FlushBuffer();
      }
      CDC_FlushBuffer();
    }
    return buffered;
  }

  RingBuffer<uint8_t, 128> receive_buffer;
  RingBuffer<uint8_t, 128> transmit_buffer;
  volatile bool host_connected;
};

extern CDCSerial UsbSerial;

#endif //_CDCSERIAL_H_
