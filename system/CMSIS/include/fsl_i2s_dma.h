/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FSL_I2S_DMA_H_
#define _FSL_I2S_DMA_H_

#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "fsl_flexcomm.h"

#include "fsl_dma.h"
#include "fsl_i2s.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup i2s_dma_driver
 * @{
 */

/*! @name Driver version */
/*@{*/
/*! @brief I2S DMA driver version 2.2.2. */
#define FSL_I2S_DMA_DRIVER_VERSION (MAKE_VERSION(2, 2, 2))
/*@}*/

/*! @brief Members not to be accessed / modified outside of the driver. */
typedef struct _i2s_dma_handle i2s_dma_handle_t;

/*!
 * @brief Callback function invoked from DMA API on completion.
 *
 * @param base I2S base pointer.
 * @param handle pointer to I2S transaction.
 * @param completionStatus status of the transaction.
 * @param userData optional pointer to user arguments data.
 */
typedef void (*i2s_dma_transfer_callback_t)(I2S_Type *base,
                                            i2s_dma_handle_t *handle,
                                            status_t completionStatus,
                                            void *userData);
/*! @brief i2s dma handle */
struct _i2s_dma_handle
{
    uint32_t state;                                    /*!< Internal state of I2S DMA transfer */
    uint8_t bytesPerFrame;                             /*!< bytes per frame */
    i2s_dma_transfer_callback_t completionCallback;    /*!< Callback function pointer */
    void *userData;                                    /*!< Application data passed to callback */
    dma_handle_t *dmaHandle;                           /*!< DMA handle */
    volatile i2s_transfer_t i2sQueue[I2S_NUM_BUFFERS]; /*!< Transfer queue storing transfer buffers */
    volatile uint8_t queueUser;                        /*!< Queue index where user's next transfer will be stored */
    volatile uint8_t queueDriver;                      /*!< Queue index of buffer actually used by the driver */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*! @} */

/*!
 * @name DMA API
 * @{
 */

/*!
 * @brief Initializes handle for transfer of audio data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param dmaHandle pointer to dma handle structure.
 * @param callback function to be called back when transfer is done or fails.
 * @param userData pointer to data passed to callback.
 */
void I2S_TxTransferCreateHandleDMA(I2S_Type *base,
                                   i2s_dma_handle_t *handle,
                                   dma_handle_t *dmaHandle,
                                   i2s_dma_transfer_callback_t callback,
                                   void *userData);

/*!
 * @brief Begins or queue sending of the given data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param transfer data buffer.
 *
 * @retval kStatus_Success
 * @retval kStatus_I2S_Busy if all queue slots are occupied with unsent buffers.
 */
status_t I2S_TxTransferSendDMA(I2S_Type *base, i2s_dma_handle_t *handle, i2s_transfer_t transfer);

/*!
 * @brief Aborts transfer of data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 */
void I2S_TransferAbortDMA(I2S_Type *base, i2s_dma_handle_t *handle);

/*!
 * @brief Initializes handle for reception of audio data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param dmaHandle pointer to dma handle structure.
 * @param callback function to be called back when transfer is done or fails.
 * @param userData pointer to data passed to callback.
 */
void I2S_RxTransferCreateHandleDMA(I2S_Type *base,
                                   i2s_dma_handle_t *handle,
                                   dma_handle_t *dmaHandle,
                                   i2s_dma_transfer_callback_t callback,
                                   void *userData);

/*!
 * @brief Begins or queue reception of data into given buffer.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param transfer data buffer.
 *
 * @retval kStatus_Success
 * @retval kStatus_I2S_Busy if all queue slots are occupied with buffers
 *         which are not full.
 */
status_t I2S_RxTransferReceiveDMA(I2S_Type *base, i2s_dma_handle_t *handle, i2s_transfer_t transfer);

/*!
 * @brief Invoked from DMA interrupt handler.
 *
 * @param handle pointer to DMA handle structure.
 * @param userData argument for user callback.
 * @param transferDone if transfer was done.
 * @param tcds
 */
void I2S_DMACallback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);

/*! @} */

/*! @} */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_I2S_DMA_H_ */
