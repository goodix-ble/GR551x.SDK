/**
 ****************************************************************************************
 *
 * @file gr55xx_hal_qspi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of QSPI HAL library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
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

/** @defgroup HAL_QSPI QSPI
  * @brief QSPI HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_QSPI_H__
#define __GR55xx_HAL_QSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_spi.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_QSPI_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_QSPI_state HAL QSPI state
  * @{
  */

/**
  * @brief HAL QSPI State Enumerations definition
  */
typedef enum
{
    HAL_QSPI_STATE_RESET             = 0x00,    /**< Peripheral not initialized                            */
    HAL_QSPI_STATE_READY             = 0x01,    /**< Peripheral initialized and ready for use              */
    HAL_QSPI_STATE_BUSY              = 0x02,    /**< Peripheral in indirect mode and busy                  */
    HAL_QSPI_STATE_BUSY_INDIRECT_TX  = 0x12,    /**< Peripheral in indirect mode with transmission ongoing */
    HAL_QSPI_STATE_BUSY_INDIRECT_RX  = 0x22,    /**< Peripheral in indirect mode with reception ongoing    */
    HAL_QSPI_STATE_ABORT             = 0x08,    /**< Peripheral with abort request ongoing                 */
    HAL_QSPI_STATE_ERROR             = 0x04     /**< Peripheral in error                                   */

} hal_qspi_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_QSPI_STRUCTURES Structures
  * @{
  */

/** @defgroup QSPI_Configuration QSPI Configuration
  * @{
  */

/**
  * @brief QSPI init Structure definition
  */
typedef struct _qspi_init_t
{
    uint32_t clock_prescaler;   /**< Specifies the prescaler factor for generating clock based on the AHB clock.
                                     This parameter can be a number between 0 and 0xFFFF */

    uint32_t clock_mode;        /**< Specifies the Clock Mode. It indicates the level that clock takes between commands.
                                     This parameter can be a value of @ref QSPI_Clock_Mode */
    
    uint32_t rx_sample_delay;   /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                     This parameter can be a number between 0 and 0x7 */
} qspi_init_t;
/** @} */

/** @defgroup QSPI_handle QSPI handle
  * @{
  */

/**
  * @brief QSPI handle Structure definition
  */
typedef struct _qspi_handle
{
    ssi_regs_t            *p_instance;               /**< QSPI registers base address        */

    qspi_init_t           init;                      /**< QSPI communication parameters      */

    uint8_t               *p_tx_buffer;              /**< Pointer to QSPI Tx transfer Buffer */

    __IO uint32_t         tx_xfer_size;              /**< QSPI Tx Transfer size              */

    __IO uint32_t         tx_xfer_count;             /**< QSPI Tx Transfer Counter           */

    uint8_t               *p_rx_buffer;              /**< Pointer to QSPI Rx transfer Buffer */

    __IO uint32_t         rx_xfer_size;              /**< QSPI Rx Transfer size              */

    __IO uint32_t         rx_xfer_count;             /**< QSPI Rx Transfer Counter           */

    void (*write_fifo)(struct _qspi_handle *p_qspi); /**< Pointer to QSPI Tx transfer FIFO write function */

    void (*read_fifo)(struct _qspi_handle *p_qspi);  /**< Pointer to QSPI Rx transfer FIFO read function  */

    dma_handle_t          *p_dma;                    /**< QSPI Rx/Tx DMA Handle parameters   */

    __IO hal_lock_t       lock;                      /**< Locking object                     */

    __IO hal_qspi_state_t state;                     /**< QSPI communication state           */

    __IO uint32_t         error_code;                /**< QSPI Error code                    */

    uint32_t              timeout;                   /**< Timeout for the QSPI memory access */

    uint32_t              retention[9];              /**< DMA important register information. */
} qspi_handle_t;
/** @} */

/** @defgroup QSPI_Command QSPI command
  * @{
  */

/**
  * @brief QSPI command Structure definition
  */
typedef struct _qspi_command_t
{
    uint32_t instruction;               /**< Specifies the Instruction to be sent.
                                             This parameter can be a value (8-bit) between 0x00 and 0xFF. */

    uint32_t address;                   /**< Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize).
                                             This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF. */

    uint32_t instruction_size;          /**< Specifies the Instruction Size.
                                             This parameter can be a value of @ref QSPI_Instruction_Size. */

    uint32_t address_size;              /**< Specifies the Address Size.
                                             This parameter can be a value of @ref QSPI_Address_Size. */

    uint32_t dummy_cycles;              /**< Specifies the Number of Dummy Cycles.
                                             This parameter can be a number between 0 and 31. */

    uint32_t data_size;                /**< Specifies the QSPI address width.
                                             This parameter can be a value of @ref QSPI_Data_Size. */

    uint32_t instruction_address_mode;  /**< Specifies the Instruction and Address Mode.
                                             This parameter can be a value of @ref QSPI_Inst_Addr_Mode. */

    uint32_t data_mode;                 /**< Specifies the Data Mode (used for dummy cycles and data phases).
                                             This parameter can be a value of @ref QSPI_Data_Mode. */

    uint32_t length;                    /**< Specifies the number of data to transfer. (This is the number of bytes).
                                             This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length
                                             until end of memory).  */

} qspi_command_t;
/** @} */

/** @} */

/** @addtogroup HAL_QSPI_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_QSPI_Callback Callback
  * @{
  */

/**
  * @brief HAL_QSPI Callback function definition
  */

typedef struct _hal_qspi_callback
{
    void (*qspi_msp_init)(qspi_handle_t *p_qspi);                   /**< QSPI init MSP callback                 */
    void (*qspi_msp_deinit)(qspi_handle_t *p_qspi);                 /**< QSPI de-init MSP callback              */
    void (*qspi_error_callback)(qspi_handle_t *p_qspi);             /**< QSPI error callback                    */
    void (*qspi_abort_cplt_callback)(qspi_handle_t *p_qspi);        /**< QSPI abort complete callback           */
    void (*qspi_fifo_threshold_callback)(qspi_handle_t *p_qspi);    /**< QSPI FIFO threshold callback           */
    void (*qspi_rx_cplt_callback)(qspi_handle_t *p_qspi);           /**< QSPI rx transfer completed callback    */
    void (*qspi_tx_cplt_callback)(qspi_handle_t *p_qspi);           /**< QSPI tx transfer completed callback    */
} hal_qspi_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_QSPI_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup QSPI_Exported_Constants QSPI Exported Constants
  * @{
  */

/** @defgroup QSPI_Error_Code QSPI Error Code
  * @{
  */
#define HAL_QSPI_ERROR_NONE             ((uint32_t)0x00000000) /**< No error                 */
#define HAL_QSPI_ERROR_TIMEOUT          ((uint32_t)0x00000001) /**< Timeout error            */
#define HAL_QSPI_ERROR_TRANSFER         ((uint32_t)0x00000002) /**< Transfer error           */
#define HAL_QSPI_ERROR_DMA              ((uint32_t)0x00000004) /**< DMA transfer error       */
#define HAL_QSPI_ERROR_INVALID_PARAM    ((uint32_t)0x00000008) /**< Invalid parameter error */
/** @} */

/** @defgroup QSPI_Clock_Mode QSPI Clock Mode
  * @{
  */
#define QSPI_CLOCK_MODE_0               (LL_SSI_SCPOL_LOW | LL_SSI_SCPHA_1EDGE)   /**< Inactive state of CLK is low;
                                                                                       CLK toggles at the start of the first data bit   */
#define QSPI_CLOCK_MODE_1               (LL_SSI_SCPOL_LOW | LL_SSI_SCPHA_2EDGE)   /**< Inactive state of CLK is low;
                                                                                       CLK toggles in the middle of the first data bit  */
#define QSPI_CLOCK_MODE_2               (LL_SSI_SCPOL_HIGH | LL_SSI_SCPHA_1EDGE)  /**< Inactive state of CLK is high;
                                                                                       CLK toggles at the start of the first data bit   */
#define QSPI_CLOCK_MODE_3               (LL_SSI_SCPOL_HIGH | LL_SSI_SCPHA_2EDGE)  /**< Inactive state of CLK is high;
                                                                                       CLK toggles in the middle of the first data bit  */
/** @} */

/** @defgroup QSPI_Data_Mode QSPI Data Mode
  * @{
  */
#define QSPI_DATA_MODE_SPI              LL_SSI_FRF_SPI          /**< Standard SPI Frame Format  */
#define QSPI_DATA_MODE_DUALSPI          LL_SSI_FRF_DUALSPI      /**< Dual SPI Frame Format      */
#define QSPI_DATA_MODE_QUADSPI          LL_SSI_FRF_QUADSPI      /**< Quad SPI Frame Format      */
/** @} */

/** @defgroup QSPI_Instruction_Size QSPI Instruction Size
  * @{
  */
#define QSPI_INSTSIZE_00_BITS           LL_SSI_INSTSIZE_0BIT    /**< 0-bit (No Instruction) */
#define QSPI_INSTSIZE_04_BITS           LL_SSI_INSTSIZE_4BIT    /**< 4-bit Instruction      */
#define QSPI_INSTSIZE_08_BITS           LL_SSI_INSTSIZE_8BIT    /**< 8-bit Instruction      */
#define QSPI_INSTSIZE_16_BITS           LL_SSI_INSTSIZE_16BIT   /**< 16-bit Instruction     */
/** @} */

/** @defgroup QSPI_Address_Size QSPI Address Size
  * @{
  */
#define QSPI_ADDRSIZE_00_BITS           LL_SSI_ADDRSIZE_0BIT    /**< 0-bit  address */
#define QSPI_ADDRSIZE_04_BITS           LL_SSI_ADDRSIZE_4BIT    /**< 4-bit  address */
#define QSPI_ADDRSIZE_08_BITS           LL_SSI_ADDRSIZE_8BIT    /**< 8-bit  address */
#define QSPI_ADDRSIZE_12_BITS           LL_SSI_ADDRSIZE_12BIT   /**< 12-bit address */
#define QSPI_ADDRSIZE_16_BITS           LL_SSI_ADDRSIZE_16BIT   /**< 16-bit address */
#define QSPI_ADDRSIZE_20_BITS           LL_SSI_ADDRSIZE_20BIT   /**< 20-bit address */
#define QSPI_ADDRSIZE_24_BITS           LL_SSI_ADDRSIZE_24BIT   /**< 24-bit address */
#define QSPI_ADDRSIZE_28_BITS           LL_SSI_ADDRSIZE_28BIT   /**< 28-bit address */
#define QSPI_ADDRSIZE_32_BITS           LL_SSI_ADDRSIZE_32BIT   /**< 32-bit address */
/** @} */

/** @defgroup QSPI_Data_Size Data Width
  * @{
  */
#define QSPI_DATASIZE_04_BITS           LL_SSI_DATASIZE_4BIT    /**< Data length for SPI transfer:  4 bits */
#define QSPI_DATASIZE_05_BITS           LL_SSI_DATASIZE_5BIT    /**< Data length for SPI transfer:  5 bits */
#define QSPI_DATASIZE_06_BITS           LL_SSI_DATASIZE_6BIT    /**< Data length for SPI transfer:  6 bits */
#define QSPI_DATASIZE_07_BITS           LL_SSI_DATASIZE_7BIT    /**< Data length for SPI transfer:  7 bits */
#define QSPI_DATASIZE_08_BITS           LL_SSI_DATASIZE_8BIT    /**< Data length for SPI transfer:  8 bits */
#define QSPI_DATASIZE_09_BITS           LL_SSI_DATASIZE_9BIT    /**< Data length for SPI transfer:  9 bits */
#define QSPI_DATASIZE_10_BITS           LL_SSI_DATASIZE_10BIT   /**< Data length for SPI transfer:  10 bits */
#define QSPI_DATASIZE_11_BITS           LL_SSI_DATASIZE_11BIT   /**< Data length for SPI transfer:  11 bits */
#define QSPI_DATASIZE_12_BITS           LL_SSI_DATASIZE_12BIT   /**< Data length for SPI transfer:  12 bits */
#define QSPI_DATASIZE_13_BITS           LL_SSI_DATASIZE_13BIT   /**< Data length for SPI transfer:  13 bits */
#define QSPI_DATASIZE_14_BITS           LL_SSI_DATASIZE_14BIT   /**< Data length for SPI transfer:  14 bits */
#define QSPI_DATASIZE_15_BITS           LL_SSI_DATASIZE_15BIT   /**< Data length for SPI transfer:  15 bits */
#define QSPI_DATASIZE_16_BITS           LL_SSI_DATASIZE_16BIT   /**< Data length for SPI transfer:  16 bits */
#define QSPI_DATASIZE_17_BITS           LL_SSI_DATASIZE_17BIT   /**< Data length for SPI transfer:  17 bits */
#define QSPI_DATASIZE_18_BITS           LL_SSI_DATASIZE_18BIT   /**< Data length for SPI transfer:  18 bits */
#define QSPI_DATASIZE_19_BITS           LL_SSI_DATASIZE_19BIT   /**< Data length for SPI transfer:  19 bits */
#define QSPI_DATASIZE_20_BITS           LL_SSI_DATASIZE_20BIT   /**< Data length for SPI transfer:  20 bits */
#define QSPI_DATASIZE_21_BITS           LL_SSI_DATASIZE_21BIT   /**< Data length for SPI transfer:  21 bits */
#define QSPI_DATASIZE_22_BITS           LL_SSI_DATASIZE_22BIT   /**< Data length for SPI transfer:  22 bits */
#define QSPI_DATASIZE_23_BITS           LL_SSI_DATASIZE_23BIT   /**< Data length for SPI transfer:  23 bits */
#define QSPI_DATASIZE_24_BITS           LL_SSI_DATASIZE_24BIT   /**< Data length for SPI transfer:  24 bits */
#define QSPI_DATASIZE_25_BITS           LL_SSI_DATASIZE_25BIT   /**< Data length for SPI transfer:  25 bits */
#define QSPI_DATASIZE_26_BITS           LL_SSI_DATASIZE_26BIT   /**< Data length for SPI transfer:  26 bits */
#define QSPI_DATASIZE_27_BITS           LL_SSI_DATASIZE_27BIT   /**< Data length for SPI transfer:  27 bits */
#define QSPI_DATASIZE_28_BITS           LL_SSI_DATASIZE_28BIT   /**< Data length for SPI transfer:  28 bits */
#define QSPI_DATASIZE_29_BITS           LL_SSI_DATASIZE_29BIT   /**< Data length for SPI transfer:  29 bits */
#define QSPI_DATASIZE_30_BITS           LL_SSI_DATASIZE_30BIT   /**< Data length for SPI transfer:  30 bits */
#define QSPI_DATASIZE_31_BITS           LL_SSI_DATASIZE_31BIT   /**< Data length for SPI transfer:  31 bits */
#define QSPI_DATASIZE_32_BITS           LL_SSI_DATASIZE_32BIT   /**< Data length for SPI transfer:  32 bits */

/** @} */


/** @defgroup QSPI_Inst_Addr_Mode QSPI Instruction and Address Mode
  * @{
  */
#define QSPI_INST_ADDR_ALL_IN_SPI       LL_SSI_INST_ADDR_ALL_IN_SPI         /**< Instruction and address are sent in SPI mode */
#define QSPI_INST_IN_SPI_ADDR_IN_SPIFRF LL_SSI_INST_IN_SPI_ADDR_IN_SPIFRF   /**< Instruction is sent in SPI mode, and address is sent in Daul/Quad SPI mode */
#define QSPI_INST_ADDR_ALL_IN_SPIFRF    LL_SSI_INST_ADDR_ALL_IN_SPIFRF      /**< Instruction and address are sent in Daul/Quad SPI mode */
/** @} */

/** @defgroup QSPI_Flags QSPI Flags
  * @{
  */
#define QSPI_FLAG_DCOL                  LL_SSI_SR_DCOL          /**< Data collision error flag  */
#define QSPI_FLAG_TXE                   LL_SSI_SR_TXE           /**< Transmission error flag    */
#define QSPI_FLAG_RFF                   LL_SSI_SR_RFF           /**< Rx FIFO full flag          */
#define QSPI_FLAG_RFNE                  LL_SSI_SR_RFNE          /**< Rx FIFO not empty flag     */
#define QSPI_FLAG_TFE                   LL_SSI_SR_TFE           /**< Tx FIFO empty flag         */
#define QSPI_FLAG_TFNF                  LL_SSI_SR_TFNF          /**< Tx FIFO not full flag      */
#define QSPI_FLAG_BUSY                  LL_SSI_SR_BUSY          /**< Busy flag                  */
/** @} */

/** @defgroup QSPI_Interrupts QSPI Interrupts
  * @{
  */
#define QSPI_IT_MST                     LL_SSI_IS_MST           /**< Multi-Master Contention Interrupt flag */
#define QSPI_IT_RXF                     LL_SSI_IS_RXF           /**< Receive FIFO Full Interrupt flag       */
#define QSPI_IT_RXO                     LL_SSI_IS_RXO           /**< Receive FIFO Overflow Interrupt flag   */
#define QSPI_IT_RXU                     LL_SSI_IS_RXU           /**< Receive FIFO Underflow Interrupt flag  */
#define QSPI_IT_TXO                     LL_SSI_IS_TXO           /**< Transmit FIFO Overflow Interrupt flag  */
#define QSPI_IT_TXE                     LL_SSI_IS_TXE           /**< Transmit FIFO Empty Interrupt flag     */
/** @} */

/** @defgroup QSPI_Timeout_definition QSPI Timeout_definition
  * @{
  */
#define HAL_QSPI_TIMEOUT_DEFAULT_VALUE ((uint32_t)5000)         /**< 5s */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup QSPI_Exported_Macros QSPI Exported Macros
  * @{
  */

/** @brief  Reset QSPI handle states.
  * @param  __HANDLE__ QSPI handle.
  * @retval None
  */
#define __HAL_QSPI_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_QSPI_STATE_RESET)

/** @brief  Enable the specified QSPI peripheral.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->SSI_EN, SSI_SSIEN_EN)

/** @brief  Disable the specified QSPI peripheral.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->SSI_EN, SSI_SSIEN_EN)

/** @brief  Enable the QSPI DMA TX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_ENABLE_DMATX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->DMAC, SSI_DMAC_TDMAE)

/** @brief  Enable the QSPI DMA RX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_ENABLE_DMARX(__HANDLE__)                     SET_BITS((__HANDLE__)->p_instance->DMAC, SSI_DMAC_RDMAE)

/** @brief  Disable the QSPI DMA TX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_DISABLE_DMATX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->DMAC, SSI_DMAC_TDMAE)

/** @brief  Disable the QSPI DMA RX Request.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_DISABLE_DMARX(__HANDLE__)                    CLEAR_BITS((__HANDLE__)->p_instance->DMAC, SSI_DMAC_RDMAE)

/** @brief  Enable the specified QSPI interrupts.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref QSPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref QSPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref QSPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval None
  */
#define __HAL_QSPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)         SET_BITS((__HANDLE__)->p_instance->INTMASK, (__INTERRUPT__))

/** @brief  Disable the specified QSPI interrupts.
  * @param  __HANDLE__ Specifies the QSPI handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref QSPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref QSPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref QSPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval None
  */
#define __HAL_QSPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)        CLEAR_BITS((__HANDLE__)->p_instance->INTMASK, (__INTERRUPT__))

/** @brief  Check whether the specified QSPI interrupt source is enabled or not.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __INTERRUPT__ Specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg @ref QSPI_IT_MST Multi-Master Contention Interrupt enable
  *            @arg @ref QSPI_IT_RXF Receive FIFO Full Interrupt enable
  *            @arg @ref QSPI_IT_RXO Receive FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_RXU Receive FIFO Underflow Interrupt enable
  *            @arg @ref QSPI_IT_TXO Transmit FIFO Overflow Interrupt enable
  *            @arg @ref QSPI_IT_TXE Transmit FIFO Empty Interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_QSPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (READ_BITS((__HANDLE__)->p_instance->INTSTAT, (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Check whether the specified QSPI flag is set or not.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __FLAG__ Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_FLAG_DCOL Data collision error flag
  *            @arg @ref QSPI_FLAG_TXE  Transmission error flag
  *            @arg @ref QSPI_FLAG_RFF  Rx FIFO full flag
  *            @arg @ref QSPI_FLAG_RFNE Rx FIFO not empty flag
  *            @arg @ref QSPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref QSPI_FLAG_TFNF Tx FIFO not full flag
  *            @arg @ref QSPI_FLAG_BUSY Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_QSPI_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0) ? SET : RESET)

/** @brief  Clear the specified QSPI flag.
  * @param  __HANDLE__ Specifies the QSPI Handle.
  * @param  __FLAG__ Specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref QSPI_FLAG_DCOL Data collision error flag
  *            @arg @ref QSPI_FLAG_TXE  Transmission error flag
  *            @arg @ref QSPI_FLAG_RFF  Rx FIFO full flag
  *            @arg @ref QSPI_FLAG_RFNE Rx FIFO not empty flag
  *            @arg @ref QSPI_FLAG_TFE  Tx FIFO empty flag
  *            @arg @ref QSPI_FLAG_TFNF Tx FIFO not full flag
  *            @arg @ref QSPI_FLAG_BUSY Busy flag
  * @retval None
  */
#define __HAL_QSPI_CLEAR_FLAG(__HANDLE__, __FLAG__)             READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__))

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup QSPI_Private_Macro QSPI Private Macros
  * @{
  */

/** @brief  Check if QSPI Clock Prescaler is valid.
  * @param  __PRESCALER__ QSPI Clock Prescaler.
  * @retval SET (__PRESCALER__ is valid) or RESET (__PRESCALER__ is invalid)
  */
#define IS_QSPI_CLOCK_PRESCALER(__PRESCALER__)  ((__PRESCALER__) <= 0xFFFF)

/** @brief  Check if QSPI FIFO Threshold is valid.
  * @param  __THR__ QSPI FIFO Threshold.
  * @retval SET (__THR__ is valid) or RESET (__THR__ is invalid)
  */
#define IS_QSPI_FIFO_THRESHOLD(__THR__)         (((__THR__) >= 0) && ((__THR__) <= 7))

/** @brief  Check if QSPI Clock Mode is valid.
  * @param  __CLKMODE__ QSPI Clock Mode.
  * @retval SET (__CLKMODE__ is valid) or RESET (__CLKMODE__ is invalid)
  */
#define IS_QSPI_CLOCK_MODE(__CLKMODE__)         (((__CLKMODE__) == QSPI_CLOCK_MODE_0) || \
                                                 ((__CLKMODE__) == QSPI_CLOCK_MODE_1) || \
                                                 ((__CLKMODE__) == QSPI_CLOCK_MODE_2) || \
                                                 ((__CLKMODE__) == QSPI_CLOCK_MODE_3))

/** @brief  Check if QSPI Instruction Size is valid.
  * @param  __INST_SIZE__ QSPI Instruction Size.
  * @retval SET (__INST_SIZE__ is valid) or RESET (__INST_SIZE__ is invalid)
  */
#define IS_QSPI_INSTRUCTION_SIZE(__INST_SIZE__) (((__INST_SIZE__) == QSPI_INSTSIZE_00_BITS) || \
                                                 ((__INST_SIZE__) == QSPI_INSTSIZE_04_BITS) || \
                                                 ((__INST_SIZE__) == QSPI_INSTSIZE_08_BITS) || \
                                                 ((__INST_SIZE__) == QSPI_INSTSIZE_16_BITS))

/** @brief  Check if QSPI Address Size is valid.
  * @param  __ADDR_SIZE__ QSPI Address Size .
  * @retval SET (__ADDR_SIZE__ is valid) or RESET (__ADDR_SIZE__ is invalid)
  */
#define IS_QSPI_ADDRESS_SIZE(__ADDR_SIZE__)     (((__ADDR_SIZE__) == QSPI_ADDRSIZE_00_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_04_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_08_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_12_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_16_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_20_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_24_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_28_BITS) || \
                                                 ((__ADDR_SIZE__) == QSPI_ADDRSIZE_32_BITS))

/** @brief  Check if QSPI Dummy Cycle is valid.
  * @param  __DCY__ QSPI Dummy Cycle.
  * @retval SET (__DCY__ is valid) or RESET (__DCY__ is invalid)
  */
#define IS_QSPI_DUMMY_CYCLES(__DCY__)           ((__DCY__) <= 31)

/** @brief  Check if QSPI Instruction and Address Mode is valid.
  * @param  __MODE__ QSPI Instruction and Address Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_QSPI_INSTADDR_MODE(__MODE__)         (((__MODE__) == QSPI_INST_ADDR_ALL_IN_SPI)       || \
                                                 ((__MODE__) == QSPI_INST_IN_SPI_ADDR_IN_SPIFRF) || \
                                                 ((__MODE__) == QSPI_INST_ADDR_ALL_IN_SPIFRF))

/** @brief  Check if QSPI Data Mode is valid.
  * @param  __MODE__ QSPI Data Mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_QSPI_DATA_MODE(__MODE__)             (((__MODE__) == QSPI_DATA_MODE_SPI)     || \
                                                 ((__MODE__) == QSPI_DATA_MODE_DUALSPI) || \
                                                 ((__MODE__) == QSPI_DATA_MODE_QUADSPI))

/** @} */

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_QSPI_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup QSPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the QSPIx peripheral:

      (+) User must implement hal_qspi_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_qspi_init() to configure the selected device with
          the selected configuration:
        (++) Clock Prescaler
        (++) Clock Mode

      (+) Call the function hal_qspi_deinit() to restore the default configuration
          of the selected QSPIx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the QSPI according to the specified parameters
 *         in the qspi_init_t and initialize the associated handle.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_init(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the QSPI peripheral.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_deinit(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Initialize the QSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_qspi_msp_deinit can be implemented in the user file.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_msp_init(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  De-initialize the QSPI MSP.
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_qspi_msp_deinit can be implemented in the user file.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_msp_deinit(qspi_handle_t *p_qspi);

/** @} */

/** @defgroup QSPI_Exported_Functions_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the QSPI
    data transfers.

    [..] The QSPI supports master and slave mode:

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts.
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated QSPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The hal_qspi_tx_cplt_callback(), hal_qspi_rx_cplt_callback() and hal_qspi_txrx_cplt_callback() user callbacks
            will be executed respectively at the end of the transmit or Receive process.
            The hal_qspi_error_callback() user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1 Line (simplex) and 2 Lines (full duplex) modes.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in blocking mode.
 * @note   This function is used only in Indirect Write Mode. Dummy cycles in command will be ignored.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in blocking mode.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_receive(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit only instruction in blocking mode.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in blocking mode with standard SPI.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_transmit(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in blocking mode with standard SPI.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 * @param[in]  timeout: Timeout duration
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_receive(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode. Dummy cycles in command will be ignored.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_receive_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief Transmit an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_transmit_it(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief Receive an amount of data in non-blocking mode at standard SPI with Interrupt.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_receive_it(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data with the specified instruction and address in non-blocking mode with DMA .
 * @note   This function is used only in Indirect Write Mode. Dummy cycles in command will be ignored.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[in]  p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_transmit_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data with the specified instruction, address and dummy cycles in non-blocking mode with DMA .
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @param[out] p_data: Pointer to data buffer
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_receive_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief  Transmit instruction in non-blocking mode with DMA.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_cmd: Pointer to a qspi_command_t structure that contains the instruction and address for data transfer.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_command_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd);

/**
 ****************************************************************************************
 * @brief  Transmit an amount of data in non-blocking mode at standard SPI with DMA.
 * @note   This function is used only in Indirect Write Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be sent in bytes,  ranging between 0 and 4095.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_transmit_dma(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Receive an amount of data in non-blocking mode at standard SPI with DMA.
 * @note   This function is used only in Indirect Read Mode.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[out] p_data: Pointer to data buffer
 * @param[in]  length: Amount of data to be received in bytes
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_receive_dma(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length);

/**
 ****************************************************************************************
 * @brief  Abort the current transmission.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_abort(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Abort the current transmission (non-blocking function)
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_abort_it(qspi_handle_t *p_qspi);

/** @} */

/** @addtogroup QSPI_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle QSPI interrupt request.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_irq_handler(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Tx Transfer completed callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_tx_cplt_callback(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Rx Transfer completed callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_rx_cplt_callback(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  QSPI error callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_error_callback(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  QSPI Abort Complete callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_abort_cplt_callback(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  FIFO Threshold callback.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 ****************************************************************************************
 */
void hal_qspi_fifo_threshold_callback(qspi_handle_t *p_qspi);

/** @} */

/** @defgroup QSPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   QSPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the QSPI.
     (+) hal_qspi_get_state() API can be helpful to check in run-time the state of the QSPI peripheral.
     (+) hal_qspi_get_error() check in run-time Errors occurring during communication.
     (+) hal_qspi_set_timeout() set the timeout during internal process.
     (+) hal_qspi_set_tx_fifo_threshold() set the TX FIFO Threshold.
     (+) hal_qspi_set_rx_fifo_threshold() set the RX FIFO Threshold.
     (+) hal_qspi_get_tx_fifo_threshold() get the TX FIFO Threshold.
     (+) hal_qspi_get_rx_fifo_threshold() get the RX FIFO Threshold.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the QSPI handle state.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @retval ::HAL_QSPI_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_QSPI_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_QSPI_STATE_BUSY: Peripheral in indirect mode and busy.
 * @retval ::HAL_QSPI_STATE_BUSY_INDIRECT_TX: Peripheral in indirect mode with transmission ongoing.
 * @retval ::HAL_QSPI_STATE_BUSY_INDIRECT_RX: Peripheral in indirect mode with reception ongoing.
 * @retval ::HAL_QSPI_STATE_ABORT: Peripheral with abort request ongoing.
 * @retval ::HAL_QSPI_STATE_ERROR: Peripheral in error.
 ****************************************************************************************
 */
hal_qspi_state_t hal_qspi_get_state(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Return the QSPI error code.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @return QSPI error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_qspi_get_error(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Set the QSPI internal process timeout value.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  timeout: Internal process timeout value.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
void hal_qspi_set_timeout(qspi_handle_t *p_qspi, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Set the TX FIFO threshold.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  threshold: TX FIFO threshold value ranging between 0x0U and 0x7U.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_set_tx_fifo_threshold(qspi_handle_t *p_qspi, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Set the RX FIFO threshold.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @param[in]  threshold: RX FIFO threshold value ranging between 0x0U and 0x7U.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_set_rx_fifo_threshold(qspi_handle_t *p_qspi, uint32_t threshold);

/**
 ****************************************************************************************
 * @brief  Get the TX FIFO threshold.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @return TX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_qspi_get_tx_fifo_threshold(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Get the RX FIFO threshold.
 * @param[in]  p_qspi: Pointer to a QSPI handle which contains the configuration information for the specified QSPI module.
 * @return RX FIFO threshold
 ****************************************************************************************
 */
uint32_t hal_qspi_get_rx_fifo_threshold(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to QSPI configuration before sleep.
 * @param[in] p_qspi: Pointer to a QSPIhandle which contains the configuration
 *                 information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_suspend_reg(qspi_handle_t *p_qspi);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to QSPI configuration after sleep.
 *         This function must be used in conjunction with the hal_qspi_suspend_reg().
 * @param[in] p_qspi: Pointer to a QSPI handle which contains the configuration
 *                 information for the specified QSPI module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_qspi_resume_reg(qspi_handle_t *p_qspi);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_QSPI_H__ */

/** @} */

/** @} */

/** @} */
