/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_spi_v2.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of SPI HAL library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2020 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_SPI SPI
  * @brief SPI HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_SPI_V2_H__
#define __GR55xx_HAL_SPI_V2_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_spi.h"
#include "gr55xx_hal_def.h"
#include "gr55xx_hal_dma.h"
#include "gr55xx_hal_spi.h"

/**
  * @defgroup  HAL_SPI_MACRO Defines
  * @{
  */
#define SPI_XFER_INST_NONE                    0xFF         /**< SPI XFER Instruction None. */
#define SPI_XFER_ADDR_NONE                    0xFFFFFF     /**< SPI XFER Address None. */
#define SPI_XFER_INST_ADDR_NONE               0xFFFFFFFF   /**< SPI XFER Instruction Address None. */
/** @} */


/** @addtogroup HAL_SPI_DRIVER_FUNCTIONS Functions
  * @{
  */
 /** @defgroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPIx peripheral:

      (+) User must implement hal_spi_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_spi_init() to configure the selected device with
          the selected configuration:
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) BaudRate Prescaler
        (++) TIMode
        (++) Slave Select

      (+) Call the function hal_spi_deinit() to restore the default configuration
          of the selected SPIx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the SPI according to the specified parameters
 *         in the spi_init_t and initialize the associated handle.
 * @param[in]  p_spi:           Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  rx_sample_delay: Receive sample delay [0, 7]
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_init(spi_handle_t *p_spi, uint32_t rx_sample_delay);

/**
 ****************************************************************************************
 * @brief  De-initialize the SPI peripheral.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_deinit(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Initialize the SPI MSP.
 * @note   This function should not be modified. When the callback is needed,
           the hal_spi_msp_deinit can be implemented in the user file.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_v2_msp_init(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  De-initialize the SPI MSP.
 * @note   This function should not be modified. When the callback is needed,
           the hal_spi_msp_deinit can be implemented in the user file.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_v2_msp_deinit(spi_handle_t *p_spi);
/** @} */

/**
 ****************************************************************************************
 * @brief  set receive sample delay
 * @param[in]  p_spi:    Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  rx_delay: 0 ~ 7.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
void hal_spi_v2_set_rx_delay(spi_handle_t *p_spi, uint32_t rx_delay);


/** @defgroup SPI_Exported_Functions_Group2 IO operation functions
 *  @brief   Data transfer functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the SPI
    data transfer.

    [..] The SPI supports master and slave mode:

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The hal_spi_tx_cplt_callback(), hal_spi_rx_cplt_callback() and hal_spi_txrx_cplt_callback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The hal_spi_error_callback() user callback will be executed when a communication error is detected.

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1-Line (simplex) and 2-Line (full duplex) modes.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode with 8bit data width, and CS Signal will be assert/de-assert in every byte.
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  p_data:  Pointer to data buffer
 * @param[in]  length:  Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_transmit_8bit_toggle(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode with 8bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:    Instruction used in transfer
 * @param[in]  addr:    Address used in transfer
 * @param[in]  p_data:  Pointer to data buffer
 * @param[in]  length:  Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_transmit_8bit(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode with 8bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:    Instruction used in transfer
 * @param[in]  addr:    Address used in transfer
 * @param[in]  p_data:  Pointer to data buffer
 * @param[in]  length:  Amount of data to be received in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_receive_8bit(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit Then Receive an amount of data in blocking mode with 8bit data width.
 *         This Function works at EEPROM Read Mode, not Full duplex Mode
 * @param[in]  p_spi:     Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  tx_data:   Pointer to transmit data buffer
 * @param[in]  tx_length: Amount of data to be transmited in bytes
 * @param[in]  rx_data:   Pointer to received data buffer
 * @param[in]  rx_length: Amount of data to be received in bytes
 * @param[in]  timeout:   Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_transmit_receive_8bit(spi_handle_t *p_spi, uint8_t *tx_data, uint32_t tx_length, uint8_t *rx_data, uint32_t rx_length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode with 32bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:    Instruction used in transfer
 * @param[in]  addr:    Address used in transfer
 * @param[in]  p_data:  Pointer to data buffer
 * @param[in]  length:  Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_transmit_32bit(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout);


/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode with 32bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:   Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:    Instruction used in transfer
 * @param[in]  addr:    Address used in transfer
 * @param[in]  p_data:  Pointer to data buffer
 * @param[in]  length:  Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_receive_32bit(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in DMA mode with 8bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:  Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:   Instruction used in transfer
 * @param[in]  addr:   Address used in transfer
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_transmit_8bit_dma(spi_handle_t *p_spi, uint8_t inst, uint32_t addr,  uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in DMA mode with 8bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:  Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:   Instruction used in transfer
 * @param[in]  addr:   Address used in transfer
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_receive_8bit_dma(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in DMA mode with 32bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:     Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:      Instruction used in transfer
 * @param[in]  addr:      Address used in transfer
 * @param[in]  p_tx_data: Pointer to data buffer
 * @param[in]  length:    Amount of data to be sent in bytes
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_transmit_32bit_dma(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_tx_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in DMA mode with 32bit data width.
 *         if set inst to SPI_XFER_INST_NONE or set addr to SPI_XFER_ADDR_NONE, inst & addr won't be used in transfer
 * @param[in]  p_spi:  Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  inst:   Instruction used in transfer
 * @param[in]  addr:   Address used in transfer
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_receive_32bit_dma(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Abort ongoing transfer (blocking mode).
 * @param[in]  p_spi: SPI handle.
 * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
 *         started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable SPI Interrupts (depending of transfer direction)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort (in case of transfer in DMA mode)
 *           - Set handle State to READY
 * @note   This procedure is executed in blocking mode: when exiting function, Abort is considered as completed.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_abort(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Abort ongoing transfer (Interrupt mode).
 * @param[in]  p_spi: SPI handle.
 * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
 *         started in Interrupt or DMA mode.
 *         This procedure performs following operations :
 *           - Disable SPI Interrupts (depending of transfer direction)
 *           - Disable the DMA transfer in the peripheral register (if enabled)
 *           - Abort DMA transfer by calling hal_dma_abort_it (in case of transfer in DMA mode)
 *           - Set handle State to READY
 *           - At abort completion, call user abort complete callback
 * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
 *         considered as completed only when user abort complete callback is executed (not when exiting function).
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_abort_it(spi_handle_t *p_spi);
/** @} */


/** @addtogroup SPI_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle SPI interrupt request.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_v2_irq_handler(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Tx Transfer completed callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_v2_tx_cplt_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Rx Transfer completed callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_v2_rx_cplt_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Tx and Rx Transfer completed callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_v2_tx_rx_cplt_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  SPI error callback.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 ****************************************************************************************
 */
void hal_spi_v2_error_callback(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  SPI Abort Completed callback.
 * @param[in]  p_spi: SPI handle.
 ****************************************************************************************
 */
void hal_spi_v2_abort_cplt_callback(spi_handle_t *p_spi);

/** @} */

/** @defgroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SPI.
     (+) hal_spi_get_state() API can be helpful to check in run-time the state of the SPI peripheral
     (+) hal_spi_get_error() check in run-time Errors occurring during communication
     (+) hal_spi_set_timeout() set the timeout during internal process
     (+) hal_spi_get_tx_fifo_threshold() get the TX FIFO Threshold
     (+) hal_spi_get_rx_fifo_threshold() get the RX FIFO Threshold
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the SPI handle state.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @retval ::HAL_SPI_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_SPI_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_SPI_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_SPI_STATE_BUSY_TX: Data Transmission process is ongoing.
 * @retval ::HAL_SPI_STATE_BUSY_RX: Data Reception process is ongoing.
 * @retval ::HAL_SPI_STATE_BUSY_TX_RX: Data Transmission and Reception process is ongoing.
 * @retval ::HAL_SPI_STATE_ABORT: Peripheral with abort request ongoing.
 * @retval ::HAL_SPI_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_spi_state_t hal_spi_v2_get_state(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Return the SPI error code.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @return SPI error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_spi_v2_get_error(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Set the SPI internal process timeout value.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @param[in]  timeout: Internal process timeout value.
 ****************************************************************************************
 */
void hal_spi_v2_set_timeout(spi_handle_t *p_spi, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Get the TX FIFO threshold.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @return TX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_spi_v2_get_tx_fifo_threshold(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Get the RX FIFO threshold.
 * @param[in]  p_spi: Pointer to an SPI handle which contains the configuration information for the specified SPI module.
 * @return RX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_spi_v2_get_rx_fifo_threshold(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to SPI configuration before sleep.
 * @param[in] p_spi: Pointer to a SPI handle which contains the configuration
 *                 information for the specified SPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_suspend_reg(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to SPI configuration after sleep.
 *         This function must be used in conjunction with the hal_spi_suspend_reg().
 * @param[in] p_spi: Pointer to a SPI handle which contains the configuration
 *                 information for the specified SPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_resume_reg(spi_handle_t *p_spi);

/**
 ****************************************************************************************
 * @brief  Using DMA to Transmit data by toggling CS in every data beat.
 *
 * @param[in]  p_spi:     Pointer to a SPI handle which contains the configuration
 *                        information for the specified SPI module.
 * @param[in]  data_size: Optional value - @ref SPI_DATASIZE_8BIT  @ref SPI_DATASIZE_16BIT  @ref SPI_DATASIZE_32BIT
 * @param[in]  p_data:    Pointer to data buffer
 * @param[in]  length:    Length of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
hal_status_t hal_spi_v2_transmit_dma_with_toggle(spi_handle_t *p_spi, uint32_t data_size, uint8_t *p_data, uint32_t length);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_SPI_V2_H__ */

/** @} */

/** @} */

/** @} */
