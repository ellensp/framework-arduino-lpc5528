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

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include <fsl_i2c.h>
  #include "fsl_iocon.h"

}

#include <Wire.h>

#ifndef USEDI2CDEV_M
  #define USEDI2CDEV_M 2
#endif

#if (USEDI2CDEV_M == 0)
  #define I2CDEV_M LPC_I2C0
#elif (USEDI2CDEV_M == 1)
  #define I2CDEV_M LPC_I2C1
#elif (USEDI2CDEV_M == 2)
  #define I2CDEV_M LPC_I2C2
#elif (USEDI2CDEV_M == 3)
  #define I2CDEV_M LPC_I2C2
#elif (USEDI2CDEV_M == 4)
  #define I2CDEV_M I2C4  
#else
  #error "Master I2C device not defined!"
#endif

// Initialize Class Variables //////////////////////////////////////////////////

uint8_t TwoWire::rxBuffer[BUFFER_LENGTH];
uint8_t TwoWire::rxBufferIndex = 0;
uint8_t TwoWire::rxBufferLength = 0;

uint8_t TwoWire::txAddress = 0;
uint8_t TwoWire::txBuffer[BUFFER_LENGTH];
uint8_t TwoWire::txBufferIndex = 0;
uint8_t TwoWire::txBufferLength = 0;

uint8_t TwoWire::transmitting = 0;

I2C_Type *temp_i2c = I2C0;


// Constructors ////////////////////////////////////////////////////////////////

TwoWire::TwoWire() {
}

// Public Methods //////////////////////////////////////////////////////////////

void TwoWire::begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  i2c_master_config_t masterConfig;
  reset_ip_name_t RST_SHIFT_RSTn = kFC4_RST_SHIFT_RSTn;
  clock_attach_id_t clock_id;
  

  /*
   * Init I2C pin connect
   * I2C0 -->> SCL：P0_25  SDA: P0_24
   * I2C1 -->> SCL：P1_11  SDA: P1_10
//   * I2C2 -->> SCL：P1_25  SDA: P1_24
   * I2C2 -->> SCL：P1_27  SDA: P1_26
   * I2C3 -->> SCL：P0_02  SDA: P0_01
   * I2C4 -->> SCL：P1_20  SDA: P1_21
   * I2C5 -->> SCL：P0_09  SDA: P0_08
   * I2C6 -->> SCL：P0_12  SDA: P0_11
   * I2C7 -->> SCL：P0_19  SDA: P0_20

   */
  #if USEDI2CDEV_M == 0
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port0_pin25_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
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
    /* PORT0 PIN25 (coords: ) is configured as FC0_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 25U, port0_pin25_config);

    const uint32_t port0_pin24_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
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
    /* PORT1 PIN21 (coords: ) is configured as FC4_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 24U, port0_pin24_config);

    RST_SHIFT_RSTn = kFC0_RST_SHIFT_RSTn;
    temp_i2c = I2C0;
    clock_id = kFRO12M_to_FLEXCOMM0;

  #endif

  #if USEDI2CDEV_M == 1
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port1_pin11_config = (/* Pin is configured as FC1_TXD_SCL_MISO_WS */
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
    IOCON_PinMuxSet(IOCON, 1U, 11U, port1_pin11_config);

    const uint32_t port1_pin10_config = (/* Pin is configured as FC1_RXD_SDA_MOSI_DATA */
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
    IOCON_PinMuxSet(IOCON, 1U, 10U, port1_pin10_config);

    RST_SHIFT_RSTn = kFC1_RST_SHIFT_RSTn;
    temp_i2c = I2C1;
    clock_id = kFRO12M_to_FLEXCOMM1;
  #endif

  #if USEDI2CDEV_M == 2
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port1_pin27_config = (/* Pin is configured as FC2_TXD_SCL_MISO_WS */
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
    IOCON_PinMuxSet(IOCON, 1U, 27U, port1_pin27_config);

    const uint32_t port1_pin26_config = (/* Pin is configured as FC2_RXD_SDA_MOSI_DATA */
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
    IOCON_PinMuxSet(IOCON, 1U, 26U, port1_pin26_config);

    RST_SHIFT_RSTn = kFC2_RST_SHIFT_RSTn;
    temp_i2c = I2C2;
    clock_id = kFRO12M_to_FLEXCOMM2;
  #endif

  #if USEDI2CDEV_M == 3
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port0_pin02_config = (/* Pin is configured as FC3_TXD_SCL_MISO_WS */
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
    IOCON_PinMuxSet(IOCON, 1U, 02U, port1_pin02_config);

    const uint32_t port0_pin01_config = (/* Pin is configured as FC3_RXD_SDA_MOSI_DATA */
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
    IOCON_PinMuxSet(IOCON, 0U, 01U, port1_pin01_config);

    RST_SHIFT_RSTn = kFC3_RST_SHIFT_RSTn;
    temp_i2c = I2C3;
    clock_id = kFRO12M_to_FLEXCOMM3;
  #endif

  #if USEDI2CDEV_M == 4
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port1_pin20_config = (/* Pin is configured as FC4_TXD_SCL_MISO_WS */
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
    /* PORT1 PIN20 (coords: 4) is configured as FC4_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 1U, 20U, port1_pin20_config);

    const uint32_t port1_pin21_config = (/* Pin is configured as FC4_RXD_SDA_MOSI_DATA */
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
    /* PORT1 PIN21 (coords: 30) is configured as FC4_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 1U, 21U, port1_pin21_config);
    RST_SHIFT_RSTn = kFC4_RST_SHIFT_RSTn;
    temp_i2c = I2C4;
    clock_id = kFRO12M_to_FLEXCOMM4;   
  #endif

  #if USEDI2CDEV_M == 5
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port0_pin09_config = (/* Pin is configured as FC5_TXD_SCL_MISO_WS */
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
    /* PORT0 PIN9 (coords: ) is configured as FC5_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 09U, port0_pin09_config);

    const uint32_t port0_pin08_config = (/* Pin is configured as FC4_RXD_SDA_MOSI_DATA */
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
    /* PORT0 PIN08 (coords: ) is configured as FC5_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 08U, port0_pin08_config);
    RST_SHIFT_RSTn = kFC5_RST_SHIFT_RSTn;
    temp_i2c = I2C5;
    clock_id = kFRO12M_to_FLEXCOMM5;   
  #endif

  #if USEDI2CDEV_M == 6
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port0_pin12_config = (/* Pin is configured as FC6_TXD_SCL_MISO_WS */
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
    /* PORT0 PIN12 (coords: ) is configured as FC5_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 12U, port0_pin12_config);

    const uint32_t port0_pin11_config = (/* Pin is configured as FC6_RXD_SDA_MOSI_DATA */
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
    /* PORT0 PIN11 (coords: ) is configured as FC6_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 11U, port0_pin11_config);
    RST_SHIFT_RSTn = kFC6_RST_SHIFT_RSTn;
    temp_i2c = I2C6;
    clock_id = kFRO12M_to_FLEXCOMM6;   
  #endif
  #if USEDI2CDEV_M == 7
  /* Enables the clock for the I/O controller.: Enable Clock. */
  CLOCK_EnableClock(kCLOCK_Iocon);
  const uint32_t port0_pin19_config = (/* Pin is configured as FC6_TXD_SCL_MISO_WS */
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
    /* PORT0 PIN19 (coords: ) is configured as FC7_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 19U, port0_pin19_config);

    const uint32_t port0_pin20_config = (/* Pin is configured as FC6_RXD_SDA_MOSI_DATA */
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
    /* PORT0 PIN20 (coords: ) is configured as FC7_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 20U, port0_pin20_config);
    RST_SHIFT_RSTn = kFC7_RST_SHIFT_RSTn;
    temp_i2c = I2C7;
    clock_id = kFRO12M_to_FLEXCOMM7;   
  #endif


  /* attach 12 MHz clock to FLEXCOMM8 (I2C master) */
  CLOCK_AttachClk(clock_id);
  /* reset FLEXCOMM for I2C */
  RESET_PeripheralReset(RST_SHIFT_RSTn);
  // Initialize I2C peripheral
  I2C_MasterGetDefaultConfig(&masterConfig);

  /* Change the default baudrate configuration */
  masterConfig.baudRate_Bps = I2C_BAUDRATE;

  /* Initialize the I2C master peripheral */
  I2C_MasterInit(temp_i2c, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);


}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  // clamp to buffer length
  if (quantity > BUFFER_LENGTH)
    quantity = BUFFER_LENGTH;

  // perform blocking read into buffer
  i2c_master_transfer_t i2c_xfer;
  
  i2c_xfer.slaveAddress = address;
  i2c_xfer.direction = kI2C_Read; // kI2C_Read
  i2c_xfer.data = rxBuffer;
  i2c_xfer.dataSize = quantity;
  // i2c_xfer.subaddress = 0x1;
  i2c_xfer.subaddressSize = 0;
  i2c_xfer.flags = kI2C_TransferDefaultFlag;

  I2C_MasterTransferBlocking( temp_i2c , &i2c_xfer );

  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = quantity;

  return quantity;
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity);
}

void TwoWire::beginTransmission(uint8_t address) {
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}

uint8_t TwoWire::endTransmission(void) {
  // transmit buffer (blocking)
  i2c_master_transfer_t i2c_xfer;
  status_t status = kStatus_Fail;
  i2c_xfer.slaveAddress = txAddress;
  i2c_xfer.direction = kI2C_Write; // kI2C_Read  kI2C_Write
  i2c_xfer.data = txBuffer;
  i2c_xfer.dataSize = txBufferLength;
  // i2c_xfer.subaddress = 0x1;
  i2c_xfer.subaddressSize = 0;
  i2c_xfer.flags = kI2C_TransferNoStopFlag;

  status = I2C_MasterTransferBlocking( temp_i2c , &i2c_xfer );
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  // indicate that we are done transmitting
  transmitting = 0;

  return status == kStatus_Success ? 0 : 4;
}

uint8_t TwoWire::endTransmission(uint8_t stop) {
  // transmit buffer (blocking)
  i2c_master_transfer_t i2c_xfer;
  status_t status = kStatus_Fail;
  i2c_xfer.slaveAddress = txAddress;
  i2c_xfer.direction = kI2C_Write; // kI2C_Read  kI2C_Write
  i2c_xfer.data = txBuffer;
  i2c_xfer.dataSize = txBufferLength;
  // i2c_xfer.subaddress = 0x1;
  i2c_xfer.subaddressSize = 0;
  if(stop)
  {
    i2c_xfer.flags = kI2C_TransferDefaultFlag;
  }
  else {
    i2c_xfer.flags = kI2C_TransferNoStopFlag;
  }
  status = I2C_MasterTransferBlocking( temp_i2c , &i2c_xfer );
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  // indicate that we are done transmitting
  transmitting = 0;

  return status == kStatus_Success ? 0 : 4;
}


// must be called after beginTransmission(address)
size_t TwoWire::write(uint8_t data) {
  if (transmitting) {
    // don't bother if buffer is full
    if (txBufferLength >= BUFFER_LENGTH) return 0;

    // put byte in tx buffer
    txBuffer[txBufferIndex++] = data;

    // update amount in buffer
    txBufferLength = txBufferIndex;
  }

  return 1;
}

// must be called after beginTransmission(address)
size_t TwoWire::write(const uint8_t *data, size_t quantity) {
  size_t sent = 0;
  if (transmitting)
    for (sent = 0; sent < quantity; ++sent)
      if (!write(data[sent])) break;

  return sent;
}

// Must be called after requestFrom(address, numBytes)
int TwoWire::available(void) {
  return rxBufferLength - rxBufferIndex;
}

// Must be called after requestFrom(address, numBytes)
int TwoWire::read(void) {
  return rxBufferIndex < rxBufferLength ? rxBuffer[rxBufferIndex++] : -1;
}

// Must be called after requestFrom(address, numBytes)
int TwoWire::peek(void) {
  return rxBufferIndex < rxBufferLength ? rxBuffer[rxBufferIndex] : -1;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

TwoWire Wire = TwoWire();