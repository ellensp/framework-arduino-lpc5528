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

#ifndef _SPI_H_
#define _SPI_H_

#include "fsl_spi.h"
#include "LPC55S28.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_dma.h"
#include "fsl_spi_dma.h"

#define BOARD_NR_SPI 5

// DMA Variable
#define EXAMPLE_SPI_MASTER_RX_CHANNEL 2
#define EXAMPLE_SPI_MASTER_TX_CHANNEL 3
extern volatile bool isTransferCompleted;

#define MISO_PIN_CONFIG (0x07|0x20|0x00|0x0100)
#define MOSI_PIN_CONFIG (0x07|0x20|0x00|0x0100)
#define CS_PIN_CONFIG   (0x01|0x20|0x00|0x0100)
#define SCK_PIN_CONFIG  (0x07|0x20|0x00|0x0100)

typedef enum _spi_dataMode
{
    SPI_MODE0 = 0, /*!< Slave select 0 */
    SPI_MODE1 = 1, /*!< Slave select 1 */
    SPI_MODE2 = 2, /*!< Slave select 2 */
    SPI_MODE3 = 3, /*!< Slave select 3 */
} spi_dataMode_t;

/*
* the class of the SPISettings
*/
class SPISettings
{
    public:
        SPISettings(uint32_t clock,spi_shift_direction_t MsbOrLsbFirst,spi_dataMode_t SPIMode,bool dmaUse)
        {
            init_AlwaysInline(clock,MsbOrLsbFirst,SPIMode,kSPI_Data8Bits,dmaUse);               
        }
        SPISettings(uint32_t clock,spi_shift_direction_t MsbOrLsbFirst,spi_dataMode_t SPIMode,spi_data_width_t inDataSize,bool dmaUse)
        {
            init_AlwaysInline(clock,MsbOrLsbFirst,SPIMode,inDataSize,dmaUse);    
        }
        SPISettings(uint32_t clock,spi_shift_direction_t MsbOrLsbFirst,spi_dataMode_t SPIMode)
        {
            init_AlwaysInline(clock,MsbOrLsbFirst,SPIMode,kSPI_Data8Bits,false);               
        }
        SPISettings(void)
        {
            init_AlwaysInline(10000000,kSPI_MsbFirst,SPI_MODE0,kSPI_Data8Bits,false);
        }
        friend class SPIClass;  
    private:
        void init_AlwaysInline(uint32_t inClock, spi_shift_direction_t inBitOrder, spi_dataMode_t inDataMode, spi_data_width_t inDataSize,bool dmaUse)
        {
            clock    = inClock;
            bitOrder = inBitOrder;
            dataMode = inDataMode;
            dataSize = inDataSize; 
            dmaEn    = dmaUse;           
        }

        uint32_t clock;
        uint8_t CS_Choose;
        spi_data_width_t dataSize;
        spi_shift_direction_t bitOrder;
        spi_dataMode_t dataMode; 
        SPI_Type * spi_id;
        bool dmaEn;

        friend class SPIClass;
};

/*
* the class of the spi
*/
class SPIClass
{
    public:
        SPIClass(uint8_t spiPortNumber);

        void begin(void);
        void end(void);   
        void beginTransaction(class SPISettings&);
        void endTransaction();
        void setModule(uint8_t device);

        void setBitOrder(spi_shift_direction_t spi_direction);
        void setClockDivider(uint32_t baudrate);
        void setClock(uint32_t clock);
        void setDmaUse(bool dmaEn);
        void setDataMode(spi_dataMode_t mode);
        void setDataSize(spi_data_width_t ds);

        void transfer(uint8_t *buffer,uint16_t size);
        // void transfer(uint8_t val);
        uint8_t transfer(uint8_t val);
        uint16_t transfer16(uint16_t val); 
        void transfer16(uint16_t *buffer,uint16_t size);
        void dmaSend(uint16_t *buf, uint16_t length, bool minc); 
        void dmaTransfer(uint8_t *txbuffer,uint8_t *rxbuffer,uint16_t length);
        // uint8_t transfer(const uint16_t b);

        // flash
        // void TransferNotdma(uint8_t *txbuffer,uint8_t *rxbuffer,uint16_t length);
        void TransferMulflash(uint8_t *txbuffer,uint8_t *rxbuffer,uint16_t length);
        uint8_t transfer8Reflash(uint8_t val);
        // flash end

        // screen
        void transfer16Notdma(uint16_t buffer);
        void transfer16Notdma(uint16_t *buffer,uint16_t count);
        // void transfer16dma(uint16_t *buf, uint16_t length);
        void transfer16dma(uint16_t *Data,uint16_t Count,uint32_t MemoryIncrease);
        // screen end

        void usingInterrupt(void);
        void wirtebuffer(uint8_t *buffer,uint16_t size);
        void readRxbuffer(uint8_t *buffer,uint16_t size);

    private:
        class SPISettings _settings[BOARD_NR_SPI];
        class SPISettings *_currentSetting;
        void updateSettings(void);
};
extern class SPIClass SPI_4;
extern class SPIClass HS_SPI;
// extern class SPIClass SPI_flash;
extern class SPIClass SPI_wifi;
// extern class SPIClass TMC_SPI;          //冯工
extern bool DMA_inc;

#endif