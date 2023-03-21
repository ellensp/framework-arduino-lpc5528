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

#ifndef _HARDWARESERIAL_H_ 
#define _HARDWARESERIAL_H_


//#include <array>
extern "C++"{
#include <array>
#include <stdarg.h>
#include <stdio.h>
#include <Stream.h>
}
extern "C" {
  #include <fsl_usart.h>
  #include "fsl_gpio.h"
  #include "fsl_common.h"
  #include "fsl_iocon.h"
}


 #define IOCON_PIO_ASW_EN 0x0400u      /*!<@brief Analog switch is closed (enabled) */
#define IOCON_PIO_DIGITAL_EN 0x0100u  /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC0 0x00u         /*!<@brief Selects pin function 0 */
#define IOCON_PIO_FUNC1 0x01u         /*!<@brief Selects pin function 1 */
#define IOCON_PIO_FUNC2 0x02u         /*!<@brief Selects pin function 2 */
#define IOCON_PIO_FUNC3 0x03u         /*!<@brief Selects pin function 2 */
#define IOCON_PIO_FUNC5 0x05u         /*!<@brief Selects pin function 5 */
#define IOCON_PIO_FUNC7 0x07u         /*!<@brief Selects pin function 7 */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_INACT 0x00u    /*!<@brief No addition pin function */
#define IOCON_PIO_MODE_PULLUP 0x20u   /*!<@brief Selects pull-up function */
#define IOCON_PIO_MODE_PULLDOWN 0x10u   /*!<@brief Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u /*!<@brief Standard mode, output slew rate control is enabled */ 

#if !defined(SERIAL_TX_BUFFER_SIZE)
  #define SERIAL_TX_BUFFER_SIZE 64
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
  #define SERIAL_RX_BUFFER_SIZE 256
#endif

#if !defined(LPC_UART_IRQ_PRIORITY)
  #define LPC_UART_IRQ_PRIORITY 3
#endif

/*
  UART pin configuration:

  UART0 TX: P0_30    RX: P0_29
  UART1 TX: P1_11    RX: P1_10
  UART2 TX: P0_27    RX: P1_24  
  UART3 TX: P0_02    RX: P0_03
  UART4 TX: P1_20    RX: P1_21
  UART5 TX: P0_09    RX: P0_08
  UART6 TX: P1_16    RX: P1_13
  UART7 TX: P0_19    RX: P0_20

*/


// #define RING_BUFFER_SIZE 16
template <uint32_t RXB_SIZE = SERIAL_RX_BUFFER_SIZE, uint32_t TXB_SIZE = SERIAL_TX_BUFFER_SIZE>

class HardwareSerial : public Stream {
private:
  USART_Type *UARTx;

  uint32_t Baudrate;
  uint32_t Status;
  std::array<uint8_t, RXB_SIZE> RxBuffer; 
  uint32_t RxQueueWritePos;
  uint32_t RxQueueReadPos;
  std::array<uint8_t, TXB_SIZE> TxBuffer;
  uint32_t TxQueueWritePos;
  uint32_t TxQueueReadPos;
  //Add by Martin 2021.10.29
  // uint8_t demoRingBuffer[RING_BUFFER_SIZE];
  // volatile uint16_t txIndex; /* Index of the data to send out. */
  // volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

public:
  HardwareSerial(USART_Type *UARTx)
    : UARTx(UARTx)
    , Baudrate(0)
    , RxQueueWritePos(0)
    , RxQueueReadPos(0)
    , TxQueueWritePos(0)
    , TxQueueReadPos(0)
  {
  }

  void begin(uint32_t baudrate) {

    uint32_t uart_clock = CLOCK_GetFlexCommClkFreq(0U);
    IRQn_Type uart_irp = FLEXCOMM0_IRQn;
    usart_config_t config;

    if (Baudrate == baudrate) return; // No need to re-initialize
    if(UARTx == USART0)
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                            IOCON_PIO_FUNC1 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
        IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

        const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                                            IOCON_PIO_FUNC1 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
        IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);
        /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
        uart_clock = CLOCK_GetFlexCommClkFreq(0U);
        uart_irp = FLEXCOMM0_IRQn;
    }
    else if(UARTx == USART1)
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port1_pin11_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC2 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 1U, 11U, port1_pin11_config);

        const uint32_t port1_pin10_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC2 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 1U, 10U, port1_pin10_config);
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);
        uart_clock = CLOCK_GetFlexCommClkFreq(1U);
        uart_irp = FLEXCOMM1_IRQn;
    }
    else if(UARTx == USART2)
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port0_pin27_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC1 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 0U, 27U, port0_pin27_config);

        const uint32_t port1_pin24_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC1 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 1U, 24U, port1_pin24_config);
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
        uart_clock = CLOCK_GetFlexCommClkFreq(2U);
        uart_irp = FLEXCOMM2_IRQn;
    }
    else if(UARTx == USART3)
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port0_pin2_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC1 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 0U, 2U, port0_pin2_config);

        const uint32_t port0_pin3_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC1 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 0U, 3U, port0_pin3_config);
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
        uart_clock = CLOCK_GetFlexCommClkFreq(3U);
        uart_irp = FLEXCOMM3_IRQn;
    }
    else if(UARTx == USART4)
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port1_pin20_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC5 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 1U, 20U, port1_pin20_config);

        const uint32_t port1_pin21_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC5 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 1U, 21U, port1_pin21_config);
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
        uart_clock = CLOCK_GetFlexCommClkFreq(4U);
        uart_irp = FLEXCOMM4_IRQn;
    }
    else if(UARTx == USART5)
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port0_pin9_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC3 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 0U, 9U, port0_pin9_config);

        const uint32_t port0_pin8_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC3 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 0U, 8U, port0_pin8_config);
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM5);
        uart_clock = CLOCK_GetFlexCommClkFreq(5U);
        uart_irp = FLEXCOMM5_IRQn;
    }
    else if(UARTx == USART6)
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port1_pin16_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC2 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 1U, 16U, port1_pin16_config);

        const uint32_t port1_pin13_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC2 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 1U, 13U, port1_pin13_config);
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM6);
        uart_clock = CLOCK_GetFlexCommClkFreq(6U);
        uart_irp = FLEXCOMM6_IRQn;
    }
    else //UARTx = USART7
    {
        /* Enables the clock for the I/O controller.: Enable Clock. */
        CLOCK_EnableClock(kCLOCK_Iocon);
        const uint32_t port0_pin19_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC7 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 0U, 19U, port0_pin19_config);

        const uint32_t port0_pin20_config = (/* Pin is configured as  */
                                            IOCON_PIO_FUNC7 |
                                            /* No addition pin function */
                                            IOCON_PIO_MODE_INACT |
                                            /* Standard mode, output slew rate control is enabled */
                                            IOCON_PIO_SLEW_STANDARD |
                                            /* Input function is not inverted */
                                            IOCON_PIO_INV_DI |
                                            /* Enables digital function */
                                            IOCON_PIO_DIGITAL_EN |
                                            /* Open drain is disabled */
                                            IOCON_PIO_OPENDRAIN_DI);
        IOCON_PinMuxSet(IOCON, 0U, 20U, port0_pin20_config);
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM7);
        uart_clock = CLOCK_GetFlexCommClkFreq(7U);
        uart_irp = FLEXCOMM7_IRQn;
    }

    /* Initialize UART Configuration parameter structure to default state:
     * Baudrate = baudrate
     * 8 data bit
     * 1 Stop bit
     * None parity
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = baudrate;  
    config.enableTx     = true;
    config.enableRx     = true;

    USART_Init( UARTx, &config, uart_clock );  

    /* Enable RX interrupt. */
    USART_EnableInterrupts(UARTx, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable );
    EnableIRQ(uart_irp);

    RxQueueWritePos = RxQueueReadPos = 0;
    if constexpr (TXB_SIZE > 0) {
      TxQueueWritePos = TxQueueReadPos = 0;
    }

    // Save the configured baudrate
    Baudrate = baudrate;
  }

  int16_t peek() {
    int16_t byte = -1;

    // Temporarily lock out UART receive interrupts during this read so the UART receive
    // interrupt won't cause problems with the index values
    USART_DisableInterrupts(UARTx, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable );
    if (RxQueueReadPos != RxQueueWritePos)
      byte = RxBuffer[RxQueueReadPos];
    // Re-enable UART interrupts
    USART_EnableInterrupts(UARTx, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable );
    return byte;
  }

  int16_t read() {    
    int16_t byte = -1;
     
    // Temporarily lock out UART receive interrupts during this read so the UART receive
    // interrupt won't cause problems with the index values
    USART_DisableInterrupts(UARTx, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable );

    // byte = RxBuffer[RxQueueWritePos];  
    if (RxQueueReadPos != RxQueueWritePos) {
      byte = RxBuffer[RxQueueReadPos];
      RxQueueReadPos = (RxQueueReadPos + 1) % RXB_SIZE;
      // byte = RxBuffer[RxQueueWritePos];
      // RxQueueReadPos = (RxQueueReadPos + 1) % RXB_SIZE;
    }
    // Re-enable UART interrupts
    USART_EnableInterrupts(UARTx, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable );
    return byte;
  }

  size_t write(uint8_t send) {
      
    if constexpr (TXB_SIZE > 0) {
      size_t bytes = 0;

      // If the Tx Buffer is full, wait for space to clear
      if ((TxQueueWritePos+1) % TXB_SIZE == TxQueueReadPos) flushTX();

      // If the queue is empty and there's space in the FIFO, immediately send the byte
      if (TxQueueWritePos == TxQueueReadPos ) {   
        USART_WriteBlocking(UARTx, &send, 1);
        return 1; 
      }
      // Otherwiise, write the byte to the transmit buffer
      else if ((TxQueueWritePos+1) % TXB_SIZE != TxQueueReadPos) {
        TxBuffer[TxQueueWritePos] = send;
        TxQueueWritePos = (TxQueueWritePos+1) % TXB_SIZE;
        bytes++;
      }
      return bytes;
    } else {
      USART_WriteBlocking(UARTx, &send, 1);
      return 1; 
    }

  }

  size_t write(char* src, size_t length) {
    for (size_t i = 0; i < length; ++i) {
       write(src[i]);              
    }   
    return length;
  }

  void flushTX() {
    if constexpr (TXB_SIZE > 0) {
      // Wait for the tx buffer and FIFO to drain
      while (TxQueueWritePos != TxQueueReadPos && (USART_GetStatusFlags(UARTx)==kUSART_TxFifoEmptyFlag) == 1); // TX FIFO empty 2021.10.25
    }
  }

  size_t available() {
    return (RxQueueWritePos + RXB_SIZE - RxQueueReadPos) % RXB_SIZE;
  }

  void flush() {
    RxQueueWritePos = 0;
    RxQueueReadPos = 0;
  }

  size_t printf(const char *format, ...) {
    char RxBuffer[256];
    va_list vArgs;
    va_start(vArgs, format);
    int length = vsnprintf(RxBuffer, 256, format, vArgs);
    va_end(vArgs);
    if (length > 0 && length < 256) {
      for (size_t i = 0; i < (size_t)length; ++i)
        write(RxBuffer[i]);
    }
    return length;

  }
  size_t printfln(const char *format, ...) {
   
    size_t length = printf(format);
    
    write('\n');
    return length;

  }

  operator bool() { return true; }
  virtual bool recv_callback(const char c) { return true; }

  void IRQHandler() {
 
    uint8_t data;

    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(UARTx))
    {
        data = USART_ReadByte(UARTx);
        /* If ring buffer is not full, add data to ring buffer. */
        if (((RxQueueWritePos + 1) % RXB_SIZE) != RxQueueReadPos)
        {
            // demoRingBuffer[rxIndex] = data;
            RxBuffer[RxQueueWritePos] = data;
            RxQueueWritePos++;
            RxQueueWritePos %= RXB_SIZE;
        }        
    }

  }

};  



extern  HardwareSerial<> Serial;
extern  HardwareSerial<> Serial1;
extern  HardwareSerial<> Serial2;
extern  HardwareSerial<> Serial3;
extern  HardwareSerial<> Serial4;
extern  HardwareSerial<> Serial5;
extern  HardwareSerial<> Serial6;
extern  HardwareSerial<> Serial7;



#endif // _HARDWARESERIAL_H_


