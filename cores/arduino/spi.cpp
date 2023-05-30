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

#include "SPI.h"
#include "Arduino.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
class SPIClass SPI_4(3);// 1 HS_SPI   2  SPI0  3  SPI4  4 SPITest
// class SPIClass SPI_flash(4);
class SPIClass SPI_wifi(2);
class SPIClass HS_SPI(1);
class SPIClass TMC_SPI(4);// 1 HS_SPI   2  SPI0  3  SPI4  4 SPITest //冯工

uint8_t BUFFER_SIZE=64;
uint8_t srcBuff[64];
uint8_t destBuff[640];
uint32_t txIndex = BUFFER_SIZE;
uint32_t rxIndex = BUFFER_SIZE;

/*******************************************************************************
 * DMA Variable
 ******************************************************************************/
bool DMA_inc=true;
spi_dma_handle_t masterHandle;
spi_transfer_t masterXfer;
spi_transfer_16t masterXfer16;
dma_handle_t masterTxHandle;
dma_handle_t masterRxHandle;

spi_dma_handle_t masterHandleSPI0;
dma_handle_t masterTxHandleSPI0;
dma_handle_t masterRxHandleSPI0;

uint8_t DmaUserdataforSPI8=1;
volatile bool SPI0DMATransferflagDone = false;
volatile bool isTransferCompleted = false;

uint8_t masterRxDataDma[640]={0};
// uint8_t masterTxDataDma[640]={0};
uint16_t masterRxDataDma16[10]={0};
// uint8_t masterTxDataDma[640]={0};

/*******************************************************************************
 * SPI Functions
 ******************************************************************************/
extern "C"
{
   void FLEXCOMM7_IRQHandler(void)
   {
      if ((SPI_GetStatusFlags(SPI7) & kSPI_RxNotEmptyFlag) && (rxIndex > 0))
      {
         destBuff[BUFFER_SIZE - rxIndex] = SPI_ReadData(SPI7);
         rxIndex--;
      }
      if ((SPI_GetStatusFlags(SPI7) & kSPI_TxNotFullFlag) && (txIndex > 0))
      {
         if (txIndex == 1)
         {
               /* need to disable interrupts before write last data */
               SPI_DisableInterrupts(SPI7, kSPI_TxLvlIrq);
               SPI_WriteData(SPI7, (uint16_t)(srcBuff[BUFFER_SIZE - txIndex]), kSPI_FrameAssert);
         }
         else
         {
               SPI_WriteData(SPI7, (uint16_t)(srcBuff[BUFFER_SIZE - txIndex]), 0);
         }
         txIndex--;
      }
      if ((txIndex == 0U) && (rxIndex == 0U))
      {
         SPI_DisableInterrupts(SPI7, kSPI_RxLvlIrq);
      }
      SDK_ISR_EXIT_BARRIER;
   }


}

void SPIClass::wirtebuffer(uint8_t *buffer,uint16_t size)
{
   uint8_t i=0;

   BUFFER_SIZE = (size>=64?64:size);
   txIndex = size;
   rxIndex = size;
   for(i=0;i<size;i++)
   {
      srcBuff[i] = *buffer;
      buffer++;
   }
}

void SPIClass::readRxbuffer(uint8_t *buffer,uint16_t size)
{
   uint16_t i=0;
   size = (size>=300?300:size);
   for(i=0;i<size;i++)
   {
         *buffer = destBuff[i];
         buffer++;
   }
}

SPIClass::SPIClass(uint8_t device)
{
  #if BOARD_NR_SPI >= 1
    _settings[0].dataMode = SPI_MODE0;
    _settings[0].dataSize = kSPI_Data8Bits;
    _settings[0].bitOrder = kSPI_MsbFirst;
    _settings[0].dmaEn    = false;
    _settings[0].spi_id   = SPI8;
    _settings[0].CS_Choose = P1_01;
  #endif

  #if BOARD_NR_SPI >= 2
    _settings[1].dataMode = SPI_MODE0;
    _settings[1].dataSize = kSPI_Data8Bits;
    _settings[1].bitOrder = kSPI_MsbFirst;
    _settings[1].dmaEn    = false;
    _settings[1].spi_id   = SPI0;
    _settings[1].CS_Choose = P1_22;
  #endif

  #if BOARD_NR_SPI >= 3
    _settings[2].dataMode = SPI_MODE0;
    _settings[2].dataSize = kSPI_Data8Bits;
    _settings[2].bitOrder = kSPI_MsbFirst;
    _settings[2].dmaEn    = false;
    _settings[2].spi_id   = SPI4;
    _settings[2].CS_Choose = P1_18;
  #endif

  #if BOARD_NR_SPI >= 4
    _settings[3].dataMode = SPI_MODE0;
    _settings[3].dataSize = kSPI_Data8Bits;
    _settings[3].bitOrder = kSPI_MsbFirst;
    _settings[3].dmaEn    = false;
    _settings[3].spi_id   = SPI0;
    _settings[3].CS_Choose = P0_16;
  #endif

  #if BOARD_NR_SPI >= 5
    _settings[4].dataMode = SPI_MODE0;
    _settings[4].dataSize = kSPI_Data8Bits;
    _settings[4].bitOrder = kSPI_MsbFirst;
    _settings[4].dmaEn    = false;
    _settings[4].spi_id   = SPI8;
    _settings[4].CS_Choose = P1_15;
  #endif
  setModule(device);
}

void SPIClass::setClock(uint32_t clock)
{ 
   if(_currentSetting->spi_id == SPI8)
   {
      clock = (clock>=50000000?50000000:clock);
   }
   else
   {
      clock = (clock>=20000000?20000000:clock);
   }
   _currentSetting->clock = clock; 
}

void SPIClass::setDmaUse(bool dmaEn)
{
   _currentSetting->dmaEn =  dmaEn;  
}

void SPIClass::setModule(uint8_t device)
{ 
   _currentSetting = &_settings[device - 1]; 
} 

void SPIClass::setBitOrder(spi_shift_direction_t bitOrder)
{ 
   _currentSetting->bitOrder = bitOrder; 
}

void SPIClass::setDataMode(spi_dataMode_t dataMode)
{ 
   _currentSetting->dataMode = dataMode;    
}

void SPIClass::setDataSize(spi_data_width_t dataSize)
{ 
   _currentSetting->dataSize = dataSize;
}

void SPIClass::begin(void)
{
   if(_currentSetting->spi_id == SPI8)  // 1 HS_SPI   2  SPI0  3  SPI4  4 SPI_Test
   {
      gpio_pin_config_t P1_15_pinconfig = {
         kGPIO_DigitalOutput,
         1,
      };  
      // HS_SPI
      CLOCK_EnableClock(kCLOCK_Iocon);
      // MOSI
      IOCON->PIO[0][26] = ((IOCON->PIO[0][26] &(~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
                           | IOCON_PIO_FUNC(0x09u)
                           | IOCON_PIO_DIGIMODE(0x01));
      // CS 改成软件控制
      // IOCON->PIO[1][1] = ((IOCON->PIO[1][1] &(~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
      //                      | IOCON_PIO_FUNC(0x05)
      //                      | IOCON_PIO_DIGIMODE(0x01));
      // SCK
      IOCON->PIO[1][2] = ((IOCON->PIO[1][2] & (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
                           | IOCON_PIO_FUNC(0x06)
                           | IOCON_PIO_DIGIMODE(0x01));
      // MISO
      IOCON->PIO[1][3] = ((IOCON->PIO[1][3] & (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
                           | IOCON_PIO_FUNC(0x06)
                           | IOCON_PIO_DIGIMODE(0x01));
      CLOCK_DisableClock(kCLOCK_Iocon);

      // if(_currentSetting->CS_Choose == P1_01)
      // {
      //    // GPIO_PortInit(GPIO, 1);
      //    GPIO_PinInit(GPIO, 1, 1, &P1_15_pinconfig);
      //    GPIO_PinWrite(GPIO,1,1,1); 
      // }
      // else
      // {
      //    GPIO_PinInit(GPIO, 1,15, &P1_15_pinconfig);
      //    GPIO_PinWrite(GPIO,1,15,1);          
      // }

      CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 0U, true);
      CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 3U, false);
      CLOCK_AttachClk(kPLL0_DIV_to_HSLSPI);//   //kFRO12M_to_HSLSPI
      RESET_PeripheralReset(kHSLSPI_RST_SHIFT_RSTn);
   }
   if(_currentSetting->spi_id == SPI0) // 1 HS_SPI   2  SPI0  3  SPI4  4 SPI_Test
   {
      // SPI0
      gpio_pin_config_t P1_15_pinconfig = {
         kGPIO_DigitalOutput,
         1,
      };  
      CLOCK_EnableClock(kCLOCK_Iocon); 
      IOCON->PIO[0][28] = ((IOCON->PIO[0][28] &
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
                         | IOCON_PIO_FUNC(0x01)
                         | IOCON_PIO_DIGIMODE(0x01));
      const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                         0x01 |
                                         0x00 |
                                         0x00 |
                                         0x00 |
                                         0x0100|
                                         0x00);
      IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);
      const uint32_t port0_pin30_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                         0x01 |
                                         0x00 |
                                         0x00 |
                                         0x00 |
                                         0x0100|
                                         0x00);
      IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config); 
      CLOCK_DisableClock(kCLOCK_Iocon); 

      // if(_currentSetting->CS_Choose == P0_16)
      // {
      //    // GPIO_PortInit(GPIO, 1);
      //    GPIO_PinInit(GPIO, 0, 16, &P1_15_pinconfig);
      //    GPIO_PinWrite(GPIO,0,16,1); 
      // }
      // else
      // {
      //    GPIO_PinInit(GPIO, 1, 22, &P1_15_pinconfig);
      //    GPIO_PinWrite(GPIO,1,22,1);          
      // }

      // CLOCK_EnableClock(kCLOCK_Gpio1);
      // // GPIO_PortInit(GPIO, 1);
      // GPIO_PinInit(GPIO, 1, 15, &P1_15_pinconfig);

      // GPIO_PinWrite(GPIO,1,15,1);  

      // CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
      CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 0U, true);
      CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 3U, false);
      CLOCK_AttachClk(kPLL0_DIV_to_FLEXCOMM0);
      RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn); 
   }
   if(_currentSetting->spi_id == SPI4) // 1 HS_SPI   2  SPI0  3  SPI4  4 SPI_Test
   {
      gpio_pin_config_t led_config = {
         kGPIO_DigitalOutput,
         1,
      };

      CLOCK_EnableClock(kCLOCK_Iocon);  
      IOCON->PIO[1][19] = ((IOCON->PIO[1][19] &
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
                         | IOCON_PIO_FUNC(0x05)
                         | IOCON_PIO_DIGIMODE(0x01)); 
      IOCON->PIO[1][20] = ((IOCON->PIO[1][20] &
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
                         | IOCON_PIO_FUNC(0x05)
                         | IOCON_PIO_DIGIMODE(0x01));
      IOCON->PIO[1][21] = ((IOCON->PIO[1][21] &
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))
                         | IOCON_PIO_FUNC(0x05)
                         | IOCON_PIO_DIGIMODE(0x01));
      CLOCK_DisableClock(kCLOCK_Iocon);

      // CLOCK_EnableClock(kCLOCK_Gpio1);
      // GPIO_PinInit(GPIO, 1, 18, &led_config);
      // GPIO_PinWrite(GPIO,1,18,1); 

      CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
      RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);   
   }
   if(_currentSetting->spi_id == SPI7) // 1 HS_SPI   2  SPI0  3  SPI4  4 SPI_Test
   {
      //SPI_Test
      CLOCK_EnableClock(kCLOCK_Iocon);
      IOCON_PinMuxSet(IOCON, 0U, 19U, MISO_PIN_CONFIG); // MISO pin config
      IOCON_PinMuxSet(IOCON, 0U, 20U, MOSI_PIN_CONFIG); // MOSI pin config
      IOCON_PinMuxSet(IOCON, 1U, 20U, CS_PIN_CONFIG);   // CS pin config
      IOCON_PinMuxSet(IOCON, 0U, 21U, SCK_PIN_CONFIG);  // SCK pin config
      CLOCK_DisableClock(kCLOCK_Iocon);

      CLOCK_AttachClk(kFRO12M_to_FLEXCOMM7);
      RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);     
   }
   updateSettings();
}

void SPIClass::end()
{
   SPI_Deinit(_currentSetting->spi_id);
}

void SPIClass::endTransaction() {}      //冯工

void SPIClass::beginTransaction(class SPISettings&cfg)
{
  setBitOrder(cfg.bitOrder);
  setDataMode(cfg.dataMode);
  setDataSize(cfg.dataSize);
  setClock(cfg.clock);
  setDmaUse(cfg.dmaEn);
  begin();
}

void SPIClass::setClockDivider(uint32_t baudrate)
{
   SPI7->CFG &= ~(SPI_CFG_ENABLE_MASK);
   SPI_MasterSetBaud(_currentSetting->spi_id,baudrate,12000000);    
   SPI7->CFG |= SPI_CFG_ENABLE_MASK; 
}

void SPIClass::transfer16(uint16_t *buffer,uint16_t size)
{
   spi_transfer_t xfer;

   if(_currentSetting->dmaEn==true)
   {
      isTransferCompleted = false;
      masterXfer.txData      = (uint8_t *)&buffer;
      masterXfer.rxData      = (uint8_t *)masterRxDataDma;
      masterXfer.dataSize    = size * sizeof(buffer[0]);
      masterXfer.configFlags = kSPI_FrameAssert;  
      if (kStatus_Success != SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandle, &masterXfer))
      {
         return ;
      }   
      while(1)
      {
         if(isTransferCompleted == true)
         {
            isTransferCompleted = false;
            break;
         }            
      }   
   }
   else
   {
      xfer.txData = (uint8_t *)&buffer;
      xfer.rxData = destBuff;
      xfer.dataSize    = size * sizeof(buffer[0]);;
      xfer.configFlags = kSPI_FrameAssert;

      // if(_currentSetting->spi_id==SPI4 && _currentSetting->CS_Choose == P1_18)
      // {
      //    GPIO_PinWrite(GPIO,1,18,0);      
      // }
      // if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
      // {
      //    GPIO_PinWrite(GPIO,0,16,0); 
      // }
      // else
      // {
      //    GPIO_PinWrite(GPIO,1,22,0); 
      // }

      SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);

      // if(_currentSetting->spi_id==SPI4 && _currentSetting->CS_Choose == P1_18)
      // {
      //    GPIO_PinWrite(GPIO,1,18,1);      
      // }
      // if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
      // {
      //    GPIO_PinWrite(GPIO,0,16,1); 
      // }
      // else
      // {
      //    GPIO_PinWrite(GPIO,1,22,1); 
      // }
   }   
}

void SPIClass::transfer(uint8_t *buffer,uint16_t size)
{
   spi_transfer_t xfer;

   if(_currentSetting->dmaEn==true)
   {
      isTransferCompleted = false;
      masterXfer.txData      = (uint8_t *)buffer;
      masterXfer.rxData      = (uint8_t *)masterRxDataDma;
      masterXfer.dataSize    = size * sizeof(*buffer);
      masterXfer.configFlags = kSPI_FrameAssert;  
      if (kStatus_Success != SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandle, &masterXfer))
      {
         return ;
      }     
   }
   else
   {
      xfer.txData = buffer;
      xfer.rxData = destBuff;
      xfer.dataSize    = size;
      xfer.configFlags = kSPI_FrameAssert;

      // if(_currentSetting->spi_id==SPI4)
      // {
      //    GPIO_PinWrite(GPIO,1,18,0);      
      // }
      // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
      // {
      //    GPIO_PinWrite(GPIO,0,16,0); 
      // }
      // else
      // {
      //    GPIO_PinWrite(GPIO,1,22,0); 
      // }

      SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);

      // if(_currentSetting->spi_id==SPI4)
      // {
      //    GPIO_PinWrite(GPIO,1,18,1);      
      // }
      // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
      // {
      //    GPIO_PinWrite(GPIO,0,16,1); 
      // }
      // else
      // {
      //    GPIO_PinWrite(GPIO,1,22,1); 
      // }
   }
}

uint8_t SPIClass::transfer(uint8_t val)
{
   spi_transfer_t xfer;
   uint8_t Returndata=0xff;

   if(_currentSetting->dmaEn==true)
   {
      isTransferCompleted = false;
      masterXfer.txData      = &val;
      masterXfer.rxData      = &Returndata;
      masterXfer.dataSize    = 1 * sizeof(val);
      masterXfer.configFlags = kSPI_FrameAssert;  
      // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
      // {
      //    GPIO_PinWrite(GPIO,1,1,0); 
      // }

      if (kStatus_Success != SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandle, &masterXfer))
      {
         return 0;
      } 

      // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
      // {
      //    GPIO_PinWrite(GPIO,1,1,1); 
      // } 

      return Returndata;
   }
   else
   {
      xfer.txData = &val;
      xfer.rxData = &Returndata;
      xfer.dataSize    = 1 * sizeof(val);
      xfer.configFlags = kSPI_FrameAssert;

      // if(_currentSetting->spi_id==SPI4 && _currentSetting->CS_Choose == P1_18)
      // {
      //    GPIO_PinWrite(GPIO,1,18,0);      
      // }
      // if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
      // {
      //    GPIO_PinWrite(GPIO,0,16,0); 
      // }
      // if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P1_22)
      // {
      //    GPIO_PinWrite(GPIO,1,22,0); 
      // }

   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,0); 
   // }
   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_15)
   // {
   //    GPIO_PinWrite(GPIO,1,15,0); 
   // }

      SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);

   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,1); 
   // }
   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_15)
   // {
   //    GPIO_PinWrite(GPIO,1,15,1); 
   // }

   // if(_currentSetting->spi_id==SPI4 && _currentSetting->CS_Choose == P1_18)
   // {
   //    GPIO_PinWrite(GPIO,1,18,1);      
   // }
   // if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
   // {
   //    GPIO_PinWrite(GPIO,0,16,1); 
   // }

   // else
   // {
   //    GPIO_PinWrite(GPIO,1,22,1); 
   // }

   return Returndata;
   }
}

volatile bool g_Transfer_Done;
/* User callback function for DMA transfer. */
void DMA_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
        g_Transfer_Done = true;
    }
}
// for screen
// MemoryIncrease    true  is sorce increase
//                   false is source not increase
spi_transfer_t xfer;
void SPIClass::transfer16dma(uint16_t *Data,uint16_t Count,uint32_t MemoryIncrease)
{
   dma_handle_t g_DMA_Handle;
   dma_channel_config_t transferConfig;

   DMA_Deinit(DMA0);
   DMA_Init(DMA0);
   DMA_CreateHandle(&g_DMA_Handle, DMA0, 3);
   DMA_EnableChannel(DMA0, 3);
   DMA_SetCallback(&g_DMA_Handle, DMA_Callback, NULL); 

   SPI_EnableTxDMA(SPI8, true); 
   DMA_PrepareChannelTransfer(&transferConfig, Data, (uint32_t *)&(SPI8->FIFORD),
   // 4 是指 32 位的宽度  2 是指 16 位的宽度  1 是指 8 位的宽度
   // 16 是指字节数   
                               DMA_CHANNEL_XFER(false, false, true, false, 2, kDMA_AddressInterleave0xWidth,
                                                kDMA_AddressInterleave0xWidth, Count*2),
                               kDMA_StaticToStatic, NULL, NULL); 
   DMA_SubmitChannelTransfer(&g_DMA_Handle, &transferConfig);
   //  CS low P1_01
   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,0); 
   // }

    DMA_StartTransfer(&g_DMA_Handle);
    /* Wait for DMA transfer finish */
    while (g_Transfer_Done != true)
    {
    }

   //  CS High P1_01
   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,1); 
   // }

   // isTransferCompleted = false;
   // xfer.txData = (uint8_t *)&Data;
   // xfer.rxData = NULL;
   // xfer.dataSize    = Count * sizeof(Data[0]);
   // xfer.configFlags = kSPI_FrameAssert;


   // if(MemoryIncrease)
   // {
   //    // SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandle, &xfer);
   //    // while(1)
   //    // {
   //    //    if(isTransferCompleted == true)
   //    //    {
   //    //       isTransferCompleted = false;
   //    //       break;
   //    //    }            
   //    // } 
   // }
   // else
   // {
   //    // // xfer.configFlags = kSPI_FrameDelay;
   //    // SPI_MasterTransferDMA_NotInc(_currentSetting->spi_id, &masterHandle, &xfer);
   //    // while(1)
   //    // {
   //    //    if(isTransferCompleted == true)
   //    //    {
   //    //       isTransferCompleted = false;
   //    //       break;
   //    //    }            
   //    // } 
   // }



}


void SPIClass::transfer16Notdma(uint16_t buffer)
{
   spi_transfer_t xfer;

   xfer.txData = (uint8_t *)&buffer;
   xfer.rxData = NULL;
   xfer.dataSize    = 1 * sizeof(buffer);
   xfer.configFlags = kSPI_FrameAssert;

   // if(_currentSetting->spi_id==SPI4)
   // {
   //    GPIO_PinWrite(GPIO,1,18,0);      
   // }
   // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
   // {
   //    GPIO_PinWrite(GPIO,0,16,0); 
   // }
   // else
   // {
   //    GPIO_PinWrite(GPIO,1,22,0); 
   // }

   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,0); 
   // }
   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_15)
   // {
   //    GPIO_PinWrite(GPIO,1,15,0); 
   // }

   SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);

   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,1); 
   // }
   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_15)
   // {
   //    GPIO_PinWrite(GPIO,1,15,1); 
   // }

   // if(_currentSetting->spi_id==SPI4)
   // {
   //    GPIO_PinWrite(GPIO,1,18,1);      
   // }
   // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
   // {
   //    GPIO_PinWrite(GPIO,0,16,1); 
   // }
   // else
   // {
   //    GPIO_PinWrite(GPIO,1,22,1); 
   // }
}



void SPIClass::transfer16Notdma(uint16_t *buffer,uint16_t count)
{
   spi_transfer_t xfer;

   xfer.txData = (uint8_t *)buffer;
   xfer.rxData = NULL;
   xfer.dataSize    = count * sizeof(buffer[0]);
   xfer.configFlags = kSPI_FrameAssert;

   // if(_currentSetting->spi_id==SPI4)
   // {
   //    GPIO_PinWrite(GPIO,1,18,0);      
   // }
   // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
   // {
   //    GPIO_PinWrite(GPIO,0,16,0); 
   // }
   // else
   // {
   //    GPIO_PinWrite(GPIO,1,22,0); 
   // }

   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,0); 
   // }

   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_15)
   // {
   //    GPIO_PinWrite(GPIO,1,15,0); 
   // }

   SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);

   // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
   // {
   //    GPIO_PinWrite(GPIO,1,1,1); 
   // }

   // if(_currentSetting->spi_id==SPI4)
   // {
   //    GPIO_PinWrite(GPIO,1,18,1);      
   // }
   // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
   // {
   //    GPIO_PinWrite(GPIO,0,16,1); 
   // }
   // else
   // {
   //    GPIO_PinWrite(GPIO,1,22,1); 
   // }
}


// memory increase
// void SPIClass::transfer16dma(uint16_t *buf, uint16_t length)
// {
//    isTransferCompleted = false;
//    masterXfer.txData      = (uint8_t *)&buf;
//    masterXfer.rxData      = NULL;
//    masterXfer.dataSize    = length * sizeof(buf[0]);
//    masterXfer.configFlags = kSPI_FrameAssert;  
//    if (kStatus_Success !=SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandle, &masterXfer))
//    {
//       return ;   
//    }   
//    while(1)
//    {
//       if(isTransferCompleted == true)
//       {
//          isTransferCompleted = false;
//          break;
//       }            
//    }        
         
// }

// for screen end===========================

// for flash
void SPIClass::TransferMulflash(uint8_t *txbuffer,uint8_t *rxbuffer,uint16_t length)
{
   spi_transfer_t xfer;  

   xfer.txData = (uint8_t *)&txbuffer;
   // xfer.rxData = (uint8_t *)&rxbuffer;
   xfer.rxData = rxbuffer;
   xfer.dataSize    = length * sizeof(rxbuffer[0]);
   xfer.configFlags = kSPI_FrameAssert;

   if(_currentSetting->dmaEn==true)
   {
      SPI0DMATransferflagDone = false;
      if (kStatus_Success != SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandleSPI0, &xfer))
      {
         return;
      }   
      while(1)
      {
         if(SPI0DMATransferflagDone == true)
         {
            SPI0DMATransferflagDone = false;
            break;
         }            
      }       
   }
   else
   {
      SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);
   }
}



uint8_t SPIClass::transfer8Reflash(uint8_t val)
{
   spi_transfer_t xfer;
   uint8_t Returndata=0xff;

   if(_currentSetting->spi_id != SPI0 && _currentSetting->CS_Choose != P1_22)
   {
      return 0;    
   }
   xfer.txData      = &val;
   xfer.rxData      = &Returndata;
   xfer.dataSize    = 1 * sizeof(val);
   xfer.configFlags = kSPI_FrameAssert; 

   if(_currentSetting->dmaEn==true)
   {
      SPI0DMATransferflagDone = false;
      if (kStatus_Success != SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandleSPI0, &xfer))
      {
         return 0;
      }   
      return Returndata;
   }
   else
   {
      SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);
      return Returndata;
   }
}


// void SPIClass::TransferNotdma(uint8_t *txbuffer,uint8_t *rxbuffer,uint16_t length)
// {
//    spi_transfer_t xfer;  

//    xfer.txData = (uint8_t *)&txbuffer;
//    xfer.rxData = (uint8_t *)&rxbuffer;
//    xfer.dataSize    = length * sizeof(uint8_t);
//    xfer.configFlags = kSPI_FrameAssert;

//    if(_currentSetting->spi_id==SPI4 && _currentSetting->CS_Choose == P1_18)
//    {
//       GPIO_PinWrite(GPIO,1,18,0);      
//    }
//    if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
//    {
//       GPIO_PinWrite(GPIO,0,16,0); 
//    }
//    if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P1_22)
//    {
//       GPIO_PinWrite(GPIO,1,22,0); 
//    }
//    SPI_MasterTransferBlocking(_currentSetting->spi_id, &xfer);
//    if(_currentSetting->spi_id==SPI4 && _currentSetting->CS_Choose == P1_18)
//    {
//       GPIO_PinWrite(GPIO,1,18,1);      
//    }
//    if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
//    {
//       GPIO_PinWrite(GPIO,0,16,1); 
//    }
//    // if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P1_22)
//    // {
//    //    GPIO_PinWrite(GPIO,1,22,1); 
//    // }
// }


// for flash end ==================================================

void SPIClass::dmaTransfer(uint8_t *txbuffer,uint8_t *rxbuffer,uint16_t length)
{

   isTransferCompleted = false;
   masterXfer.txData      = txbuffer;
   masterXfer.rxData      = rxbuffer;
   masterXfer.dataSize    = length * sizeof(rxbuffer[0]);
   masterXfer.configFlags = kSPI_FrameAssert;  

   // if(_currentSetting->spi_id==SPI4)
   // {
   //    GPIO_PinWrite(GPIO,1,18,0);      
   // }
   // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
   // {
   //    GPIO_PinWrite(GPIO,0,16,0); 
   // }
   // else
   // {
   //    GPIO_PinWrite(GPIO,1,22,0); 
   // }

   if (kStatus_Success !=SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandle, &masterXfer))
   {
      return ;   
   }   
   while(1)
   {
      if(isTransferCompleted == true)
      {
         isTransferCompleted = false;
         break;
      }            
   } 

   // if(_currentSetting->spi_id==SPI4)
   // {
   //    GPIO_PinWrite(GPIO,1,18,1);      
   // }
   // else if(_currentSetting->spi_id==SPI0 && _currentSetting->CS_Choose == P0_16)
   // {
   //    GPIO_PinWrite(GPIO,0,16,1); 
   // }
   // else
   // {
   //    GPIO_PinWrite(GPIO,1,22,1); 
   // }    
}

void SPIClass::dmaSend(uint16_t *buf, uint16_t length, bool minc)
{
      if(minc==false && length != 1)
      {

         isTransferCompleted = false;
         masterXfer16.txData      = (uint16_t *)buf;
         masterXfer16.rxData      = NULL;
         masterXfer16.dataSize    = length * sizeof(uint16_t);
         masterXfer16.configFlags = kSPI_FrameAssert;  
         // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
         // {
         //    GPIO_PinWrite(GPIO,1,1,0); 
         // }
         if (kStatus_Success !=SPI_MasterTransferDMA16MemNotInc(_currentSetting->spi_id, &masterHandle, &masterXfer16))
         {
            return ;   
         }
         while(1)
         {
            if(isTransferCompleted == true)
            {
               isTransferCompleted = false;
               break;
            }            
         } 
         // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
         // {
         //    GPIO_PinWrite(GPIO,1,1,1); 
         // }
      }
      else
      {
         // isTransferCompleted = false;
         // masterXfer16.txData      = (uint16_t *)buf;
         // masterXfer16.rxData      = NULL;
         // masterXfer16.dataSize    = length * sizeof(uint16_t);
         // masterXfer16.configFlags = kSPI_FrameAssert;  
         // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
         // {
         //    GPIO_PinWrite(GPIO,1,1,0); 
         // }
         // if (kStatus_Success !=SPI_MasterTransferDMA16_MemInc(_currentSetting->spi_id, &masterHandle, &masterXfer16))
         // {
         //    return ;   
         // }
         // while(1)
         // {
         //    if(isTransferCompleted == true)
         //    {
         //       isTransferCompleted = false;
         //       break;
         //    }            
         // } 
         // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
         // {
         //    GPIO_PinWrite(GPIO,1,1,1); 
         // }

         isTransferCompleted = false;
         masterXfer.txData      = (uint8_t *)buf;
         masterXfer.rxData      = NULL;
         masterXfer.dataSize    = length * sizeof(buf[0]);
         masterXfer.configFlags = kSPI_FrameAssert;  
         // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
         // {
         //    GPIO_PinWrite(GPIO,1,1,0); 
         // }
         if (kStatus_Success !=SPI_MasterTransferDMA(_currentSetting->spi_id, &masterHandle, &masterXfer))
         {
            return ;   
         }   
         while(1)
         {
            if(isTransferCompleted == true)
            {
               isTransferCompleted = false;
               break;
            }            
         } 
         // if(_currentSetting->spi_id==SPI8 && _currentSetting->CS_Choose == P1_01)
         // {
         //    GPIO_PinWrite(GPIO,1,1,1); 
         // }       
      }     
}

void SPIClass::usingInterrupt(void)
{
    /* Enable interrupt, first enable slave and then master. */
    
    if(_currentSetting->spi_id == SPI0)
    {
         EnableIRQ(FLEXCOMM0_IRQn);
    }
   if(_currentSetting->spi_id == SPI4)
    {
         EnableIRQ(FLEXCOMM4_IRQn);
    }
        if(_currentSetting->spi_id == SPI8)
    {
         EnableIRQ(FLEXCOMM8_IRQn);
    }
    SPI_EnableInterrupts(_currentSetting->spi_id, kSPI_TxLvlIrq | kSPI_RxLvlIrq);
}

static void SPI_MasterUserCallback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
   if (status == kStatus_Success)
   {
         isTransferCompleted=true;
   }
}

static void SPI_MasterUserCallbackSPI0(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
   if (status == kStatus_Success)
   {
         SPI0DMATransferflagDone=true;
   }
}



// uint8_t SPIClass::transfer(const uint16_t b)
// {
//    SPI_ReadData(_currentSetting->spi_id);
//    SPI_WriteData(_currentSetting->spi_id,b,kSPI_FrameAssert);
//    // while(SPI_GetStatusFlags(_currentSetting->spi_id) == kSPI_TxNotFullFlag);
//    return SPI_ReadData(_currentSetting->spi_id);
// }

void SPIClass::updateSettings()
{
   spi_master_config_t userConfig = {0};

   SPI_MasterGetDefaultConfig(&userConfig);
   
   userConfig.sselNum = kSPI_Ssel1;
   userConfig.direction = _currentSetting->bitOrder;
   userConfig.dataWidth = _currentSetting->dataSize;
   userConfig.baudRate_Bps = _currentSetting->clock;
   userConfig.sselPol = kSPI_SpolActiveAllLow;

   switch(_currentSetting->dataMode)
   {
      case 0:
         userConfig.polarity = kSPI_ClockPolarityActiveHigh;
         userConfig.phase    = kSPI_ClockPhaseFirstEdge; 
         break;
      case 1:
         userConfig.polarity = kSPI_ClockPolarityActiveHigh;
         userConfig.phase    = kSPI_ClockPhaseSecondEdge; 
         break;
      case 2:
         userConfig.polarity = kSPI_ClockPolarityActiveLow;
         userConfig.phase    = kSPI_ClockPhaseFirstEdge; 
         break;
      case 3:
         userConfig.polarity = kSPI_ClockPolarityActiveLow;
         userConfig.phase    = kSPI_ClockPhaseSecondEdge;  
         break;
      default:
         userConfig.polarity = kSPI_ClockPolarityActiveHigh;
         userConfig.phase    = kSPI_ClockPhaseFirstEdge; 
         break;     
   }
   if(_currentSetting->spi_id == SPI8)
   {
      SPI_MasterInit(_currentSetting->spi_id, &userConfig,CLOCK_GetFlexCommClkFreq(8U));
   }
   else if(_currentSetting->spi_id == SPI0)
   {
      SPI_MasterInit(_currentSetting->spi_id, &userConfig,CLOCK_GetFlexCommClkFreq(0U));  
   }
   else
   {
      SPI_MasterInit(_currentSetting->spi_id, &userConfig,12000000); 
      // SPI_MasterInit(_currentSetting->spi_id, &userConfig,CLOCK_GetFlexCommClkFreq(8U)); 
   }

   if(_currentSetting->dmaEn == true)
   {
       /* DMA init */
        DMA_Init(DMA0);
        /* Configure the DMA channel,priority and handle. */
        if(_currentSetting->spi_id == SPI8)
        {
            DMA_EnableChannel(DMA0, EXAMPLE_SPI_MASTER_TX_CHANNEL);
            DMA_EnableChannel(DMA0, EXAMPLE_SPI_MASTER_RX_CHANNEL);
            DMA_SetChannelPriority(DMA0, EXAMPLE_SPI_MASTER_TX_CHANNEL, kDMA_ChannelPriority0);
            DMA_SetChannelPriority(DMA0, EXAMPLE_SPI_MASTER_RX_CHANNEL, kDMA_ChannelPriority1);
            DMA_CreateHandle(&masterTxHandle, DMA0, EXAMPLE_SPI_MASTER_TX_CHANNEL);
            DMA_CreateHandle(&masterRxHandle, DMA0, EXAMPLE_SPI_MASTER_RX_CHANNEL);

            SPI_MasterTransferCreateHandleDMA(_currentSetting->spi_id, &masterHandle, SPI_MasterUserCallback, NULL, &masterTxHandle,
                                            &masterRxHandle); 
        } 
        else if(_currentSetting->spi_id == SPI0 && _currentSetting->CS_Choose == P1_22)
        {
        // 5 TX channel 4 RX channel for flexcomm 0
            DMA_EnableChannel(DMA0, 5);
            DMA_EnableChannel(DMA0, 4);
            DMA_SetChannelPriority(DMA0, 5, kDMA_ChannelPriority3);
            DMA_SetChannelPriority(DMA0, 4, kDMA_ChannelPriority2);
            DMA_CreateHandle(&masterTxHandleSPI0, DMA0, 5);
            DMA_CreateHandle(&masterRxHandleSPI0, DMA0, 4);

            SPI_MasterTransferCreateHandleDMA(_currentSetting->spi_id, &masterHandleSPI0, SPI_MasterUserCallbackSPI0, NULL, &masterTxHandleSPI0,
                                            &masterRxHandleSPI0);
        }
        else{}
   }
   else
   {
     DMA_Deinit(DMA0); 
   }
}