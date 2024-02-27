/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_spi.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of SPI LL library.
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

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_SPI SPI
  * @brief SPI LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_SPI_H__
#define __GR55xx_LL_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (SPIM) || defined (SPIS) || defined (QSPI0) || defined (QSPI1)

/** @defgroup LL_SPI_DRIVER_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_LL_ES_INIT SPI Exported init structure
  * @{
  */

/**
  * @brief LL SPIM init structures definition
  */
typedef struct _ll_spim_init_t
{
    uint32_t transfer_direction;    /**< Specifies the SPI unidirectional or bidirectional data mode.
                                         This parameter can be a value of @ref SPI_LL_EC_TRANSFER_MODE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_transfer_direction().*/

    uint32_t data_size;             /**< Specifies the SPI data size.
                                         This parameter can be a value of @ref SPI_LL_EC_DATASIZE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_data_size().*/

    uint32_t clock_polarity;        /**< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_LL_EC_POLARITY.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_polarity().*/

    uint32_t clock_phase;           /**< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_LL_EC_PHASE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_phase().*/

    uint32_t slave_select;          /**< Specifies the SPI slave select.
                                         This parameter can be a value of @ref SPI_LL_EC_SLAVESELECT.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_enable_ss().*/

    uint32_t baud_rate;             /**< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                         This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
                                         @note The communication clock is derived from the master clock. The slave clock does not need to be set.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_baud_rate_prescaler().*/
} ll_spim_init_t;

/**
  * @brief  SPIS init structures definition
  */
typedef struct _ll_spis_init_t
{
    uint32_t data_size;             /**< Specifies the SPI data width.
                                         This parameter can be a value of @ref SPI_LL_EC_DATASIZE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_data_size().*/

    uint32_t clock_polarity;        /**< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_LL_EC_POLARITY.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_polarity().*/

    uint32_t clock_phase;           /**< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_LL_EC_PHASE.

                                         This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_phase().*/

} ll_spis_init_t;

/**
  * @brief  QSPI init structures definition
  */
typedef struct _ll_qspi_init_t
{
    uint32_t transfer_direction;        /**< Specifies the QSPI transfer or receive mode.
                                             This parameter can be a value of @ref SPI_LL_EC_TRANSFER_MODE.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_transfer_direction().*/

    uint32_t instruction_size;          /**< Specifies the QSPI instruction width.
                                             This parameter can be a value of @ref SPI_LL_EC_INSTRUCTIONSIZE.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_instruction_size().*/

    uint32_t address_size;              /**< Specifies the QSPI address width.
                                             This parameter can be a value of @ref SPI_LL_EC_ADDRESSSIZE.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_address_size().*/

    uint32_t inst_addr_transfer_format; /**< Specifies the QSPI instruction and address transfer format.
                                             This parameter can be a value of @ref SPI_LL_EC_ADDRINSTTRNASFERFORMAT.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_add_inst_transfer_format().*/

    uint32_t wait_cycles;               /**< Specifies the QSPI dummy clock.
                                             This parameter can be one of the following values: 0 ~ 31.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_wait_cycles().*/

    uint32_t data_size;                 /**< Specifies the SPI data width.
                                             This parameter can be a value of @ref SPI_LL_EC_DATASIZE.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_data_size().*/

    uint32_t clock_polarity;            /**< Specifies the serial clock steady state.
                                             This parameter can be a value of @ref SPI_LL_EC_POLARITY.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_polarity().*/

    uint32_t clock_phase;               /**< Specifies the clock active edge for the bit capture.
                                             This parameter can be a value of @ref SPI_LL_EC_PHASE.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_clock_phase().*/

    uint32_t baud_rate;                 /**< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                             This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
                                             @note The communication clock is derived from the master clock. The slave clock does not need to be set.

                                             This feature can be modified afterwards using unitary function @ref ll_spi_set_baud_rate_prescaler().*/

    uint32_t rx_sample_delay;           /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                             This parameter can be a number between 0 and 0x7 */
} ll_qspi_init_t;

/** @} */

/** @} */

/**
  * @defgroup  SPI_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_LL_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_SPI_ReadReg function
  * @{
  */
#define LL_SSI_SR_DCOL                      SSI_STAT_DCOL               /**< Data collision error flag      */
#define LL_SSI_SR_TXE                       SSI_STAT_TXE                /**< Transmission error flag        */
#define LL_SSI_SR_RFF                       SSI_STAT_RFF                /**< Rx FIFO full flag              */
#define LL_SSI_SR_RFNE                      SSI_STAT_RFNE               /**< Rx FIFO not empty flag         */
#define LL_SSI_SR_TFE                       SSI_STAT_TFE                /**< Tx FIFO empty flag             */
#define LL_SSI_SR_TFNF                      SSI_STAT_TFNF               /**< Tx FIFO not full flag          */
#define LL_SSI_SR_BUSY                      SSI_STAT_BUSY               /**< Busy flag                      */
/** @} */

/** @defgroup SPI_LL_EC_IT IT Defines
  * @brief    Interrupt definitions which can be used with LL_SPI_ReadReg and  LL_SPI_WriteReg functions
  * @{
  */
#define LL_SSI_IM_MST                       SSI_INTMASK_MSTIM           /**< Multi-Master Contention Interrupt enable   */
#define LL_SSI_IM_RXF                       SSI_INTMASK_RXFIM           /**< Receive FIFO Full Interrupt enable         */
#define LL_SSI_IM_RXO                       SSI_INTMASK_RXOIM           /**< Receive FIFO Overflow Interrupt  enable    */
#define LL_SSI_IM_RXU                       SSI_INTMASK_RXUIM           /**< Receive FIFO Underflow Interrupt  enable   */
#define LL_SSI_IM_TXO                       SSI_INTMASK_TXOIM           /**< Transmit FIFO Overflow Interrupt  enable   */
#define LL_SSI_IM_TXE                       SSI_INTMASK_TXEIM           /**< Transmit FIFO Empty Interrupt  enable      */

#define LL_SSI_IS_MST                       SSI_INTSTAT_MSTIS           /**< Multi-Master Contention Interrupt flag     */
#define LL_SSI_IS_RXF                       SSI_INTSTAT_RXFIS           /**< Receive FIFO Full Interrupt flag           */
#define LL_SSI_IS_RXO                       SSI_INTSTAT_RXOIS           /**< Receive FIFO Overflow Interrupt  flag      */
#define LL_SSI_IS_RXU                       SSI_INTSTAT_RXUIS           /**< Receive FIFO Underflow Interrupt  flag     */
#define LL_SSI_IS_TXO                       SSI_INTSTAT_TXOIS           /**< Transmit FIFO Overflow Interrupt  flag     */
#define LL_SSI_IS_TXE                       SSI_INTSTAT_TXEIS           /**< Transmit FIFO Empty Interrupt  flag        */

#define LL_SSI_RIS_MST                      SSI_RAW_INTSTAT_MSTIR       /**< Multi-Master Contention RAW Interrupt flag */
#define LL_SSI_RIS_RXF                      SSI_RAW_INTSTAT_RXFIR       /**< Receive FIFO Full RAW Interrupt flag       */
#define LL_SSI_RIS_RXO                      SSI_RAW_INTSTAT_RXOIR       /**< Receive FIFO Overflow RAW Interrupt  flag  */
#define LL_SSI_RIS_RXU                      SSI_RAW_INTSTAT_RXUIR       /**< Receive FIFO Underflow RAW Interrupt  flag */
#define LL_SSI_RIS_TXO                      SSI_RAW_INTSTAT_TXOIR       /**< Transmit FIFO Overflow RAW Interrupt  flag */
#define LL_SSI_RIS_TXE                      SSI_RAW_INTSTAT_TXEIR       /**< Transmit FIFO Empty RAW Interrupt  flag    */
/** @} */

/** @defgroup SPI_LL_EC_SPIFRAMEFORMAT SPI Frame Format
  * @{
  */
#define LL_SSI_FRF_SPI                      0x00000000UL                    /**< SPI frame format for transfer      */
#define LL_SSI_FRF_DUALSPI                  (1UL << SSI_CTRL0_SPIFRF_Pos)   /**< Dual-SPI frame format for transfer */
#define LL_SSI_FRF_QUADSPI                  (2UL << SSI_CTRL0_SPIFRF_Pos)   /**< Quad-SPI frame format for transfer */
/** @} */

/** @defgroup SPI_LL_EC_DATASIZE Datawidth
  * @{
  */
#define LL_SSI_DATASIZE_4BIT                (3UL << SSI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  4 bits */
#define LL_SSI_DATASIZE_5BIT                (4UL << SSI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  5 bits */
#define LL_SSI_DATASIZE_6BIT                (5UL << SSI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  6 bits */
#define LL_SSI_DATASIZE_7BIT                (6UL << SSI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  7 bits */
#define LL_SSI_DATASIZE_8BIT                (7UL << SSI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  8 bits */
#define LL_SSI_DATASIZE_9BIT                (8UL << SSI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer:  9 bits */
#define LL_SSI_DATASIZE_10BIT               (9UL << SSI_CTRL0_DFS32_Pos)    /**< Data length for SPI transfer: 10 bits */
#define LL_SSI_DATASIZE_11BIT               (10UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 11 bits */
#define LL_SSI_DATASIZE_12BIT               (11UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 12 bits */
#define LL_SSI_DATASIZE_13BIT               (12UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 13 bits */
#define LL_SSI_DATASIZE_14BIT               (13UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 14 bits */
#define LL_SSI_DATASIZE_15BIT               (14UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 15 bits */
#define LL_SSI_DATASIZE_16BIT               (15UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 16 bits */
#define LL_SSI_DATASIZE_17BIT               (16UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 17 bits */
#define LL_SSI_DATASIZE_18BIT               (17UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 18 bits */
#define LL_SSI_DATASIZE_19BIT               (18UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 19 bits */
#define LL_SSI_DATASIZE_20BIT               (19UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 20 bits */
#define LL_SSI_DATASIZE_21BIT               (20UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 21 bits */
#define LL_SSI_DATASIZE_22BIT               (21UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 22 bits */
#define LL_SSI_DATASIZE_23BIT               (22UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 23 bits */
#define LL_SSI_DATASIZE_24BIT               (23UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 24 bits */
#define LL_SSI_DATASIZE_25BIT               (24UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 25 bits */
#define LL_SSI_DATASIZE_26BIT               (25UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 26 bits */
#define LL_SSI_DATASIZE_27BIT               (26UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 27 bits */
#define LL_SSI_DATASIZE_28BIT               (27UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 28 bits */
#define LL_SSI_DATASIZE_29BIT               (28UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 29 bits */
#define LL_SSI_DATASIZE_30BIT               (29UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 30 bits */
#define LL_SSI_DATASIZE_31BIT               (30UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 31 bits */
#define LL_SSI_DATASIZE_32BIT               (31UL << SSI_CTRL0_DFS32_Pos)   /**< Data length for SPI transfer: 32 bits */
/** @} */

/** @defgroup SPI_LL_EC_MICROWIRECOMMANDSIZE MicroWire CommandSize
  * @{
  */
#define LL_SSI_MW_CMDSIZE_1BIT              0x00000000UL                    /**< CMD length for Microwire transfer:  1 bits */
#define LL_SSI_MW_CMDSIZE_2BIT              (1UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  2 bits */
#define LL_SSI_MW_CMDSIZE_3BIT              (2UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  3 bits */
#define LL_SSI_MW_CMDSIZE_4BIT              (3UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  4 bits */
#define LL_SSI_MW_CMDSIZE_5BIT              (4UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  5 bits */
#define LL_SSI_MW_CMDSIZE_6BIT              (5UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  6 bits */
#define LL_SSI_MW_CMDSIZE_7BIT              (6UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  7 bits */
#define LL_SSI_MW_CMDSIZE_8BIT              (7UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  8 bits */
#define LL_SSI_MW_CMDSIZE_9BIT              (8UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer:  9 bits */
#define LL_SSI_MW_CMDSIZE_10BIT             (9UL << SSI_CTRL0_CFS_Pos)      /**< CMD length for Microwire transfer: 10 bits */
#define LL_SSI_MW_CMDSIZE_11BIT             (10UL << SSI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 11 bits */
#define LL_SSI_MW_CMDSIZE_12BIT             (11UL << SSI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 12 bits */
#define LL_SSI_MW_CMDSIZE_13BIT             (12UL << SSI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 13 bits */
#define LL_SSI_MW_CMDSIZE_14BIT             (13UL << SSI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 14 bits */
#define LL_SSI_MW_CMDSIZE_15BIT             (14UL << SSI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 15 bits */
#define LL_SSI_MW_CMDSIZE_16BIT             (15UL << SSI_CTRL0_CFS_Pos)     /**< CMD length for Microwire transfer: 16 bits */
/** @} */

/** @defgroup SPI_LL_EC_TEST_MODE Test Mode
  * @{
  */
#define LL_SSI_NORMAL_MODE                  0x00000000UL                    /**< Normal mode for SPI transfer                           */
#define LL_SSI_TEST_MODE                    (1UL << SSI_CTRL0_SRL_Pos)      /**< Test mode for SPI transfer: Rx and Tx connected inside */
/** @} */

/** @defgroup SPI_LL_EC_SLAVEOUT_ENABLE Slave Out Enable
  * @{
  */
#define LL_SSI_SLAVE_OUTDIS                 0x00000000UL                    /**< Output enable for SPI transfer as slave    */
#define LL_SSI_SLAVE_OUTEN                  (1UL << SSI_CTRL0_SLVOE_Pos)    /**< Output disable for SPI transfer as slave   */
/** @} */

/** @defgroup SPI_LL_EC_TRANSFER_MODE Transfer Mode
  * @{
  */
#define LL_SSI_FULL_DUPLEX                  0x00000000UL                    /**< Full-Duplex mode. Rx and Tx transfer on 2 lines */
#define LL_SSI_SIMPLEX_TX                   (1UL << SSI_CTRL0_TMOD_Pos)     /**< Simplex Tx mode.  Tx transfer only on 1 line    */
#define LL_SSI_SIMPLEX_RX                   (2UL << SSI_CTRL0_TMOD_Pos)     /**< Simplex Rx mode.  Rx transfer only on 1 line    */
#define LL_SSI_READ_EEPROM                  (3UL << SSI_CTRL0_TMOD_Pos)     /**< Read EEPROM mode.  Rx transfer only on 1 line   */
/** @} */

/** @defgroup SPI_LL_EC_PHASE Clock Phase
  * @{
  */
#define LL_SSI_SCPHA_1EDGE                  0x00000000UL                    /**< First clock transition is the first data capture edge  */
#define LL_SSI_SCPHA_2EDGE                  (1UL << SSI_CTRL0_SCPHA_Pos)    /**< Second clock transition is the first data capture edge */
/** @} */

/** @defgroup SPI_LL_EC_POLARITY Clock Polarity
  * @{
  */
#define LL_SSI_SCPOL_LOW                    0x00000000UL                    /**< Clock to 0 when idle */
#define LL_SSI_SCPOL_HIGH                   (1UL << SSI_CTRL0_SCPOL_Pos)    /**< Clock to 1 when idle */
/** @} */

/** @defgroup SPI_LL_EC_PROTOCOL Serial Protocol
  * @{
  */
#define LL_SSI_PROTOCOL_MOTOROLA            0x00000000UL                /**< Motorola mode. Used as default value */
#define LL_SSI_PROTOCOL_TI                  (1UL << SSI_CTRL0_FRF_Pos)  /**< TI mode                              */
#define LL_SSI_PROTOCOL_MICROWIRE           (2UL << SSI_CTRL0_FRF_Pos)  /**< Microwire mode                       */
/** @} */

/** @defgroup SPI_LL_EC_MICROWIRECONTROL MicroWire Control
  * @{
  */
#define LL_SSI_MICROWIRE_HANDSHAKE_DIS      0x00000000UL                /**< Enable Handshake for Microwire transfer  */
#define LL_SSI_MICROWIRE_HANDSHAKE_EN       (1UL << SSI_MWC_MHS_Pos)    /**< Disable Handshake for Microwire transfer */

#define LL_SSI_MICROWIRE_RX                 0x00000000UL                /**< Rx mode. Rx transfer at Microwire mode */
#define LL_SSI_MICROWIRE_TX                 (1UL << SSI_MWC_MDD_Pos)    /**< Tx mode. Tx transfer at Microwire mode */

#define LL_SSI_MICROWIRE_NON_SEQUENTIAL     0x00000000UL                /**< Non-sequential for Microwire transfer  */
#define LL_SSI_MICROWIRE_SEQUENTIAL         (1UL << SSI_MWC_MWMOD_Pos)  /**< Sequential for Microwire transfer      */
/** @} */

/** @defgroup SPI_LL_EC_SLAVESELECT Slave Select
  * @{
  */
#define LL_SSI_SLAVE1                       SSI_SE_SLAVE1               /**< Enable slave1 select pin for SPI transfer  */
#define LL_SSI_SLAVE0                       SSI_SE_SLAVE0               /**< Enable slave0 select pin for SPI transfer  */
/** @} */

/** @defgroup SPI_LL_EC_DMA DMA Defines
  * @{
  */
#define LL_SSI_DMA_TX_DIS                   0x00000000UL                /**< Disable the transmit FIFO DMA channel */
#define LL_SSI_DMA_TX_EN                    SSI_DMAC_TDMAE              /**< Enable the transmit FIFO DMA channel  */

#define LL_SSI_DMA_RX_DIS                   0x00000000UL                /**< Disable the receive FIFO DMA channel */
#define LL_SSI_DMA_RX_EN                    SSI_DMAC_RDMAE              /**< Enable the receive FIFO DMA channel  */
/** @} */

/** @defgroup SPI_LL_EC_INSTRUCTIONSIZE QSPI Instruction Size
  * @{
  */
#define LL_SSI_INSTSIZE_0BIT                0x00000000UL                        /**< Instruction length for QSPI transfer:  0 bits */
#define LL_SSI_INSTSIZE_4BIT                (1UL << SSI_SCTRL0_INSTL_Pos)       /**< Instructoin length for QSPI transfer:  4 bits */
#define LL_SSI_INSTSIZE_8BIT                (2UL << SSI_SCTRL0_INSTL_Pos)       /**< Instructoin length for QSPI transfer:  8 bits */
#define LL_SSI_INSTSIZE_16BIT               (3UL << SSI_SCTRL0_INSTL_Pos)       /**< Instructoin length for QSPI transfer: 16 bits */
/** @} */

/** @defgroup SPI_LL_EC_ADDRESSSIZE QSPI Address Size
  * @{
  */
#define LL_SSI_ADDRSIZE_0BIT                0x00000000UL                        /**< Address length for QSPI transfer:  0 bits */
#define LL_SSI_ADDRSIZE_4BIT                (1UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer:  4 bits */
#define LL_SSI_ADDRSIZE_8BIT                (2UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer:  8 bits */
#define LL_SSI_ADDRSIZE_12BIT               (3UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 12 bits */
#define LL_SSI_ADDRSIZE_16BIT               (4UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 16 bits */
#define LL_SSI_ADDRSIZE_20BIT               (5UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 20 bits */
#define LL_SSI_ADDRSIZE_24BIT               (6UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 24 bits */
#define LL_SSI_ADDRSIZE_28BIT               (7UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 28 bits */
#define LL_SSI_ADDRSIZE_32BIT               (8UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 32 bits */
#define LL_SSI_ADDRSIZE_36BIT               (9UL << SSI_SCTRL0_ADDRL_Pos)       /**< Address length for QSPI transfer: 36 bits */
#define LL_SSI_ADDRSIZE_40BIT               (10UL << SSI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 40 bits */
#define LL_SSI_ADDRSIZE_44BIT               (11UL << SSI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 44 bits */
#define LL_SSI_ADDRSIZE_48BIT               (12UL << SSI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 48 bits */
#define LL_SSI_ADDRSIZE_52BIT               (13UL << SSI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 52 bits */
#define LL_SSI_ADDRSIZE_56BIT               (14UL << SSI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 56 bits */
#define LL_SSI_ADDRSIZE_60BIT               (15UL << SSI_SCTRL0_ADDRL_Pos)      /**< Address length for QSPI transfer: 60 bits */
/** @} */

/** @defgroup SPI_LL_EC_ADDRINSTTRNASFERFORMAT QSPI Address and Instruction Transfer Format
  * @{
  */
#define LL_SSI_INST_ADDR_ALL_IN_SPI         0x00000000UL                        /**< Instruction and address are sent in SPI mode */
#define LL_SSI_INST_IN_SPI_ADDR_IN_SPIFRF   (1UL << SSI_SCTRL0_TRANSTYPE_Pos)   /**< Instruction is in sent in SPI mode and address is sent in Daul/Quad SPI mode */
#define LL_SSI_INST_ADDR_ALL_IN_SPIFRF      (2UL << SSI_SCTRL0_TRANSTYPE_Pos)   /**< Instruction and address are sent in Daul/Quad SPI mode */
/** @} */

/** @defgroup SPI_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL SPIM InitStrcut default configuartion
  */
#define LL_SPIM_DEFAULT_CONFIG                         \
{                                                      \
    .transfer_direction  = LL_SSI_FULL_DUPLEX,         \
    .data_size           = LL_SSI_DATASIZE_8BIT,       \
    .clock_polarity      = LL_SSI_SCPOL_LOW,           \
    .clock_phase         = LL_SSI_SCPHA_1EDGE,         \
    .slave_select        = LL_SSI_SLAVE0,              \
    .baud_rate           = SystemCoreClock / 2000000,  \
}

/**
  * @brief LL SPIS InitStrcut default configuartion
  */
#define LL_SPIS_DEFAULT_CONFIG                       \
{                                                    \
    .data_size           = LL_SSI_DATASIZE_8BIT,     \
    .clock_polarity      = LL_SSI_SCPOL_LOW,         \
    .clock_phase         = LL_SSI_SCPHA_1EDGE,       \
}

/**
  * @brief LL QSPI InitStrcut default configuartion
  */
#define LL_QSPI_DEFAULT_CONFIG                             \
{                                                          \
    .transfer_direction  = LL_SSI_SIMPLEX_TX,              \
    .instruction_size    = LL_SSI_INSTSIZE_8BIT,           \
    .address_size        = LL_SSI_ADDRSIZE_24BIT,          \
    .inst_addr_transfer_format = LL_SSI_INST_ADDR_ALL_IN_SPI,\
    .wait_cycles         = 0,                              \
    .data_size           = LL_SSI_DATASIZE_8BIT,           \
    .clock_polarity      = LL_SSI_SCPOL_LOW,               \
    .clock_phase         = LL_SSI_SCPHA_1EDGE,             \
    .baud_rate           = SystemCoreClock / 1000000,      \
    .rx_sample_delay     = 0,                              \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI_LL_Exported_Macros SPI Exported Macros
  * @{
  */

/** @defgroup SPI_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in SPI register
  * @param  __instance__ SPI instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_SPI_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in SPI register
  * @param  __instance__ SPI instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_SPI_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup SPI_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable slave select toggle
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SSTEN                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_ss_toggle(ssi_regs_t *SPIx)
{
    SET_BITS(SPIx->CTRL0, SSI_CTRL0_SSTEN);
}

/**
  * @brief  Disable slave select toggle
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SSTEN                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_ss_toggle(ssi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->CTRL0, SSI_CTRL0_SSTEN);
}

/**
  * @brief  Check if slave select toggle is enabled
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SSTEN                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_ss_toggle(ssi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->CTRL0, SSI_CTRL0_SSTEN) == (SSI_CTRL0_SSTEN));
}

/**
  * @brief  Set data frame format for transmitting/receiving the data
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SPIFRF                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  frf This parameter can be one of the following values:
  *         @arg @ref LL_SSI_FRF_SPI
  *         @arg @ref LL_SSI_FRF_DUALSPI
  *         @arg @ref LL_SSI_FRF_QUADSPI
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_frame_format(ssi_regs_t *SPIx, uint32_t frf)
{
    MODIFY_REG(SPIx->CTRL0, SSI_CTRL0_SPIFRF, frf);
}

/**
  * @brief  Get data frame format for transmitting/receiving the data
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SPIFRF                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_FRF_SPI
  *         @arg @ref LL_SSI_FRF_DUALSPI
  *         @arg @ref LL_SSI_FRF_QUADSPI
  */
__STATIC_INLINE uint32_t ll_spi_get_frame_format(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SSI_CTRL0_SPIFRF));
}

/**
  * @brief  Set frame data size
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | DFS32                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_SSI_DATASIZE_4BIT
  *         @arg @ref LL_SSI_DATASIZE_5BIT
  *         @arg @ref LL_SSI_DATASIZE_6BIT
  *         @arg @ref LL_SSI_DATASIZE_7BIT
  *         @arg @ref LL_SSI_DATASIZE_8BIT
  *         @arg @ref LL_SSI_DATASIZE_9BIT
  *         @arg @ref LL_SSI_DATASIZE_10BIT
  *         @arg @ref LL_SSI_DATASIZE_11BIT
  *         @arg @ref LL_SSI_DATASIZE_12BIT
  *         @arg @ref LL_SSI_DATASIZE_13BIT
  *         @arg @ref LL_SSI_DATASIZE_14BIT
  *         @arg @ref LL_SSI_DATASIZE_15BIT
  *         @arg @ref LL_SSI_DATASIZE_16BIT
  *         @arg @ref LL_SSI_DATASIZE_17BIT
  *         @arg @ref LL_SSI_DATASIZE_18BIT
  *         @arg @ref LL_SSI_DATASIZE_19BIT
  *         @arg @ref LL_SSI_DATASIZE_20BIT
  *         @arg @ref LL_SSI_DATASIZE_21BIT
  *         @arg @ref LL_SSI_DATASIZE_22BIT
  *         @arg @ref LL_SSI_DATASIZE_23BIT
  *         @arg @ref LL_SSI_DATASIZE_24BIT
  *         @arg @ref LL_SSI_DATASIZE_25BIT
  *         @arg @ref LL_SSI_DATASIZE_26BIT
  *         @arg @ref LL_SSI_DATASIZE_27BIT
  *         @arg @ref LL_SSI_DATASIZE_28BIT
  *         @arg @ref LL_SSI_DATASIZE_29BIT
  *         @arg @ref LL_SSI_DATASIZE_30BIT
  *         @arg @ref LL_SSI_DATASIZE_31BIT
  *         @arg @ref LL_SSI_DATASIZE_32BIT
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_data_size(ssi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->CTRL0, SSI_CTRL0_DFS32, size);
}

/**
  * @brief  Get frame data size
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | DFS32                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_DATASIZE_4BIT
  *         @arg @ref LL_SSI_DATASIZE_5BIT
  *         @arg @ref LL_SSI_DATASIZE_6BIT
  *         @arg @ref LL_SSI_DATASIZE_7BIT
  *         @arg @ref LL_SSI_DATASIZE_8BIT
  *         @arg @ref LL_SSI_DATASIZE_9BIT
  *         @arg @ref LL_SSI_DATASIZE_10BIT
  *         @arg @ref LL_SSI_DATASIZE_11BIT
  *         @arg @ref LL_SSI_DATASIZE_12BIT
  *         @arg @ref LL_SSI_DATASIZE_13BIT
  *         @arg @ref LL_SSI_DATASIZE_14BIT
  *         @arg @ref LL_SSI_DATASIZE_15BIT
  *         @arg @ref LL_SSI_DATASIZE_16BIT
  *         @arg @ref LL_SSI_DATASIZE_17BIT
  *         @arg @ref LL_SSI_DATASIZE_18BIT
  *         @arg @ref LL_SSI_DATASIZE_19BIT
  *         @arg @ref LL_SSI_DATASIZE_20BIT
  *         @arg @ref LL_SSI_DATASIZE_21BIT
  *         @arg @ref LL_SSI_DATASIZE_22BIT
  *         @arg @ref LL_SSI_DATASIZE_23BIT
  *         @arg @ref LL_SSI_DATASIZE_24BIT
  *         @arg @ref LL_SSI_DATASIZE_25BIT
  *         @arg @ref LL_SSI_DATASIZE_26BIT
  *         @arg @ref LL_SSI_DATASIZE_27BIT
  *         @arg @ref LL_SSI_DATASIZE_28BIT
  *         @arg @ref LL_SSI_DATASIZE_29BIT
  *         @arg @ref LL_SSI_DATASIZE_30BIT
  *         @arg @ref LL_SSI_DATASIZE_31BIT
  *         @arg @ref LL_SSI_DATASIZE_32BIT
  */
__STATIC_INLINE uint32_t ll_spi_get_data_size(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SSI_CTRL0_DFS32));
}

/**
  * @brief  Set the length of the control word for the Microwire frame format
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | CFS                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_SSI_MW_CMDSIZE_1BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_2BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_3BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_4BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_5BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_6BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_7BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_8BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_9BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_10BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_11BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_12BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_13BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_14BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_15BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_16BIT
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_control_frame_size(ssi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->CTRL0, SSI_CTRL0_CFS, size);
}

/**
  * @brief  Get the length of the control word for the Microwire frame format
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | CFS                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_MW_CMDSIZE_1BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_2BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_3BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_4BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_5BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_6BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_7BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_8BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_9BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_10BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_11BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_12BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_13BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_14BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_15BIT
  *         @arg @ref LL_SSI_MW_CMDSIZE_16BIT
  */
__STATIC_INLINE uint32_t ll_spi_get_control_frame_size(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SSI_CTRL0_CFS));
}

/**
  * @brief  Enable SPI test mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SRL                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_test_mode(ssi_regs_t *SPIx)
{
    SET_BITS(SPIx->CTRL0, SSI_CTRL0_SRL);
}

/**
  * @brief  Disable SPI test mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SRL                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_test_mode(ssi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->CTRL0, SSI_CTRL0_SRL);
}

/**
  * @brief  Check if SPI test mode is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SRL                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_test_mode(ssi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->CTRL0, SSI_CTRL0_SRL) == (SSI_CTRL0_SRL));
}

/**
  * @brief  Enable slave output
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SLVOE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_slave_out(ssi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->CTRL0, SSI_CTRL0_SLVOE);
}

/**
  * @brief  Disable slave output
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SLVOE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_salve_out(ssi_regs_t *SPIx)
{
    SET_BITS(SPIx->CTRL0, SSI_CTRL0_SLVOE);
}

/**
  * @brief  Check if slave output is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SLVOE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_slave_out(ssi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->CTRL0, SSI_CTRL0_SLVOE) != (SSI_CTRL0_SLVOE));
}

/**
  * @brief  Set transfer direction mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | TMOD                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_SSI_FULL_DUPLEX
  *         @arg @ref LL_SSI_SIMPLEX_TX
  *         @arg @ref LL_SSI_SIMPLEX_RX
  *         @arg @ref LL_SSI_READ_EEPROM
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_transfer_direction(ssi_regs_t *SPIx, uint32_t transfer_direction)
{
    MODIFY_REG(SPIx->CTRL0, SSI_CTRL0_TMOD, transfer_direction);
}

/**
  * @brief  Get transfer direction mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | TMOD                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_FULL_DUPLEX
  *         @arg @ref LL_SSI_SIMPLEX_TX
  *         @arg @ref LL_SSI_SIMPLEX_RX
  *         @arg @ref LL_SSI_READ_EEPROM
  */
__STATIC_INLINE uint32_t ll_spi_get_transfer_direction(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SSI_CTRL0_TMOD));
}

/**
  * @brief  Set clock polarity
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SCPOL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  clock_polarity This parameter can be one of the following values:
  *         @arg @ref LL_SSI_SCPOL_LOW
  *         @arg @ref LL_SSI_SCPOL_HIGH
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_clock_polarity(ssi_regs_t *SPIx, uint32_t clock_polarity)
{
    MODIFY_REG(SPIx->CTRL0, SSI_CTRL0_SCPOL, clock_polarity);
}

/**
  * @brief  Get clock polarity
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SCPOL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_SCPOL_LOW
  *         @arg @ref LL_SSI_SCPOL_HIGH
  */
__STATIC_INLINE uint32_t ll_spi_get_clock_polarity(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SSI_CTRL0_SCPOL));
}

/**
  * @brief  Set clock phase
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SCPHA                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  clock_phase This parameter can be one of the following values:
  *         @arg @ref LL_SSI_SCPHA_1EDGE
  *         @arg @ref LL_SSI_SCPHA_2EDGE
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_clock_phase(ssi_regs_t *SPIx, uint32_t clock_phase)
{
    MODIFY_REG(SPIx->CTRL0, SSI_CTRL0_SCPHA, clock_phase);
}

/**
  * @brief  Get clock phase
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | SCPHA                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_SCPHA_1EDGE
  *         @arg @ref LL_SSI_SCPHA_2EDGE
  */
__STATIC_INLINE uint32_t ll_spi_get_clock_phase(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SSI_CTRL0_SCPHA));
}

/**
  * @brief  Set serial protocol used
  * @note   This bit should be written only when SPI is disabled (SSI_EN = 0) for correct operation.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | FRF                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  standard This parameter can be one of the following values:
  *         @arg @ref LL_SSI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_SSI_PROTOCOL_TI
  *         @arg @ref LL_SSI_PROTOCOL_MICROWIRE
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_standard(ssi_regs_t *SPIx, uint32_t standard)
{
    MODIFY_REG(SPIx->CTRL0, SSI_CTRL0_FRF, standard);
}

/**
  * @brief  Get serial protocol used
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | FRF                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_SSI_PROTOCOL_TI
  *         @arg @ref LL_SSI_PROTOCOL_MICROWIRE
  */
__STATIC_INLINE uint32_t ll_spi_get_standard(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL0, SSI_CTRL0_FRF));
}

/**
  * @brief  Set the number of data frames to be continuously received
  * @note   These bits should not be changed when communication is ongoing.
            This bits are effect when TMOD = 2b10 or 2b11.
            This bits are not effect in SPIS.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | NDF                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values: 0 ~ 65535
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_receive_size(ssi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->CTRL1, SSI_CTRL1_NDF, size);
}

/**
  * @brief  Get the number of data frames to be continuously received
  * @note   These bits should not be changed when communication is ongoing.
            This bits are effect when TMOD = 2b10 or 2b11.
            This bits are not effect in SPIS.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | NDF                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 65535
  */
__STATIC_INLINE uint32_t ll_spi_get_receive_size(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->CTRL1, SSI_CTRL1_NDF));
}

/**
  * @brief  Enable SPI peripheral
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SSI_EN               | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable(ssi_regs_t *SPIx)
{
    SET_BITS(SPIx->SSI_EN, SSI_SSIEN_EN);
}

/**
  * @brief  Disable SPI peripheral
  * @note   When disabling the SPI, follow the procedure described in the Reference Manual.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SSI_EN               | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable(ssi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->SSI_EN, SSI_SSIEN_EN);
}

/**
  * @brief  Check if SPI peripheral is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SSI_EN               | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled(ssi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->SSI_EN, SSI_SSIEN_EN) == (SSI_SSIEN_EN));
}

/**
  * @brief  Enable Handshake in Microwire mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MWC                  | MHS                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_micro_handshake(ssi_regs_t *SPIx)
{
    SET_BITS(SPIx->MWC, SSI_MWC_MHS);
}

/**
  * @brief  Disable Handshake in Microwire mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MWC                  | MHS                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_micro_handshake(ssi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->MWC, SSI_MWC_MHS);
}

/**
  * @brief  Check if Handshake in Microwire mode is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MWC                  | MHS                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_micro_handshake(ssi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->MWC, SSI_MWC_MHS) == (SSI_MWC_MHS));
}

/**
  * @brief  Set transfer direction mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MWC                  | MDD                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  transfer_direction This parameter can be one of the following values:
  *         @arg @ref LL_SSI_MICROWIRE_RX
  *         @arg @ref LL_SSI_MICROWIRE_TX
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_micro_transfer_direction(ssi_regs_t *SPIx, uint32_t transfer_direction)
{
    MODIFY_REG(SPIx->MWC, SSI_MWC_MDD, transfer_direction);
}

/**
  * @brief  Get transfer direction mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MWC                  | MDD                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_MICROWIRE_RX
  *         @arg @ref LL_SSI_MICROWIRE_TX
  */
__STATIC_INLINE uint32_t ll_spi_get_micro_transfer_direction(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->MWC, SSI_MWC_MDD));
}

/**
  * @brief  Set transfer mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MWC                  | MWMOD                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  transfer_mode This parameter can be one of the following values:
  *         @arg @ref LL_SSI_MICROWIRE_NON_SEQUENTIAL
  *         @arg @ref LL_SSI_MICROWIRE_SEQUENTIAL
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_micro_transfer_mode(ssi_regs_t *SPIx, uint32_t transfer_mode)
{
    MODIFY_REG(SPIx->MWC, SSI_MWC_MWMOD, transfer_mode);
}

/**
  * @brief  Get transfer mode in Microwire mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MWC                  | MWMOD                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_MICROWIRE_NON_SEQUENTIAL
  *         @arg @ref LL_SSI_MICROWIRE_SEQUENTIAL
  */
__STATIC_INLINE uint32_t ll_spi_get_micro_transfer_mode(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->MWC, SSI_MWC_MWMOD));
}

/**
  * @brief  Enable slave select
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SE                   | SLAVE1                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  SE | SLAVE0
  *
  * @param  SPIx SPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_SSI_SLAVE1
  *         @arg @ref LL_SSI_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_ss(ssi_regs_t *SPIx, uint32_t ss)
{
    SET_BITS(SPIx->SE, ss);
}

/**
  * @brief  Disable slave select
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SE                   | SLAVE1                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  SE | SLAVE0
  *
  * @param  SPIx SPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_SSI_SLAVE1
  *         @arg @ref LL_SSI_SLAVE0
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_ss(ssi_regs_t *SPIx, uint32_t ss)
{
    CLEAR_BITS(SPIx->SE, ss);
}

/**
  * @brief  Check if slave select is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SE                   | SLAVE1                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  SE | SLAVE0
  *
  * @param  SPIx SPI instance
  * @param  ss This parameter can be one of the following values:
  *         @arg @ref LL_SSI_SLAVE1
  *         @arg @ref LL_SSI_SLAVE0
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_ss(ssi_regs_t *SPIx, uint32_t ss)
{
    return (READ_BITS(SPIx->SE, ss) == ss);
}

/**
  * @brief  Set baud rate prescaler
  * @note   These bits should not be changed when communication is ongoing. SPI BaudRate = fPCLK/Prescaler.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | BAUD                 | SCKDIV                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  baud_rate This parameter can be one even value between 2 and 65534, if the value is 0, the SCLK is disable.
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_baud_rate_prescaler(ssi_regs_t *SPIx, uint32_t baud_rate)
{
    WRITE_REG(SPIx->BAUD, baud_rate);
}

/**
  * @brief  Get baud rate prescaler
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | BAUD                 | SCKDIV                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one even value between 2 and 65534.
  */
__STATIC_INLINE uint32_t ll_spi_get_baud_rate_prescaler(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->BAUD, SSI_BAUD_SCKDIV));
}

/**
  * @brief  Set threshold of TXFIFO that triggers an TXE event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TXFTL                | TFT                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_tx_fifo_threshold(ssi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->TX_FTL, threshold);
}

/**
  * @brief  Get threshold of TXFIFO that triggers an TXE event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TXFTL                | TFT                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_spi_get_tx_fifo_threshold(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->TX_FTL, SSI_TXFTL_TFT));
}

/**
  * @brief  Set threshold of RXFIFO that triggers an RXNE event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RXFTL                | RFT                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_rx_fifo_threshold(ssi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->RX_FTL, threshold);
}

/**
  * @brief  Get threshold of RXFIFO that triggers an RXNE event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RXFTL                | RFT                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_spi_get_rx_fifo_threshold(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->RX_FTL, SSI_RXFTL_RFT));
}

/**
  * @brief  Get FIFO Transmission Level
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TXFL                 | TXTFL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 8
  */
__STATIC_INLINE uint32_t ll_spi_get_tx_fifo_level(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->TX_FL, SSI_TXFL_TXTFL));
}

/**
  * @brief  Get FIFO reception Level
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RXFL                 | RXTFL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 8
  */
__STATIC_INLINE uint32_t ll_spi_get_rx_fifo_level(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->RX_FL, SSI_RXFL_RXTFL));
}

/**
  * @brief  Get ID code
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IDCODE               | ID                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value is const.
  */
__STATIC_INLINE uint32_t ll_spi_get_id_code(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->ID, SSI_IDCODE_ID));
}

/**
  * @brief  Get IP version
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | COMP                 | VERSION                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value is const.
  */
__STATIC_INLINE uint32_t ll_spi_get_version(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->VERSION_ID, SSI_COMP_VERSION));
}

/** @} */

/** @defgroup SPI_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTMASK              | INTMASK                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_SSI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SSI_IM_RXF
  *         @arg @ref LL_SSI_IM_RXO
  *         @arg @ref LL_SSI_IM_RXU
  *         @arg @ref LL_SSI_IM_TXO
  *         @arg @ref LL_SSI_IM_TXE
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_it(ssi_regs_t *SPIx, uint32_t mask)
{
    SET_BITS(SPIx->INTMASK, mask);
}

/**
  * @brief  Disable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTMASK              | INTMASK                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_SSI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SSI_IM_RXF
  *         @arg @ref LL_SSI_IM_RXO
  *         @arg @ref LL_SSI_IM_RXU
  *         @arg @ref LL_SSI_IM_TXO
  *         @arg @ref LL_SSI_IM_TXE
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_it(ssi_regs_t *SPIx, uint32_t mask)
{
    CLEAR_BITS(SPIx->INTMASK, mask);
}

/**
  * @brief  Check if interrupt is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTMASK              | INTMASK                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_SSI_IM_MST(not effect in SPIS)
  *         @arg @ref LL_SSI_IM_RXF
  *         @arg @ref LL_SSI_IM_RXO
  *         @arg @ref LL_SSI_IM_RXU
  *         @arg @ref LL_SSI_IM_TXO
  *         @arg @ref LL_SSI_IM_TXE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_it(ssi_regs_t *SPIx, uint32_t mask)
{
    return (READ_BITS(SPIx->INTMASK, mask) == mask);
}

/** @} */

/** @defgroup SPI_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get SPI status
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT                 | STAT                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_SSI_SR_DCOL(no effect in SPIS)
  *         @arg @ref LL_SSI_SR_TXE
  *         @arg @ref LL_SSI_SR_RFF
  *         @arg @ref LL_SSI_SR_RFNE
  *         @arg @ref LL_SSI_SR_TFE
  *         @arg @ref LL_SSI_SR_TFNF
  *         @arg @ref LL_SSI_SR_BUSY
  */
__STATIC_INLINE uint32_t ll_spi_get_status(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->STAT));
}

/**
  * @brief  Check active flag
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT                 | DCOL                              |
  *  +----------------------+-----------------------------------+
  *  | STAT                 | TXE                               |
  *  +----------------------+-----------------------------------+
  *  | STAT                 | RFF                               |
  *  +----------------------+-----------------------------------+
  *  | STAT                 | RFNE                              |
  *  +----------------------+-----------------------------------+
  *  | STAT                 | TFE                               |
  *  +----------------------+-----------------------------------+
  *  | STAT                 | TFNF                              |
  *  +----------------------+-----------------------------------+
  *  | STAT                 | BUSY                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_SSI_SR_DCOL(no effect in SPIS)
  *         @arg @ref LL_SSI_SR_TXE
  *         @arg @ref LL_SSI_SR_RFF
  *         @arg @ref LL_SSI_SR_RFNE
  *         @arg @ref LL_SSI_SR_TFE
  *         @arg @ref LL_SSI_SR_TFNF
  *         @arg @ref LL_SSI_SR_BUSY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_active_flag(ssi_regs_t *SPIx, uint32_t flag)
{
    return (READ_BITS(SPIx->STAT, flag) == (flag));
}

/**
  * @brief  Get SPI interrupt flags
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | INTSTAT                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_SSI_IS_MST(no effect in SPIS)
  *         @arg @ref LL_SSI_IS_RXF
  *         @arg @ref LL_SSI_IS_RXO
  *         @arg @ref LL_SSI_IS_RXU
  *         @arg @ref LL_SSI_IS_TXO
  *         @arg @ref LL_SSI_IS_TXE
  */
__STATIC_INLINE uint32_t ll_spi_get_it_flag(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->INTSTAT));
}

/**
  * @brief  Check interrupt flag
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | MSTIS                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  INTSTAT | RXFIS
  *  INTSTAT | RXOIS
  *  INTSTAT | RXUIS
  *  INTSTAT | TXOIS
  *  INTSTAT | TXEIS
  *
  * @param  SPIx SPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_SSI_IS_MST(no effect in SPIS)
  *         @arg @ref LL_SSI_IS_RXF
  *         @arg @ref LL_SSI_IS_RXO
  *         @arg @ref LL_SSI_IS_RXU
  *         @arg @ref LL_SSI_IS_TXO
  *         @arg @ref LL_SSI_IS_TXE
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_it_flag(ssi_regs_t *SPIx, uint32_t flag)
{
    return (READ_BITS(SPIx->INTSTAT, flag) == flag);
}

/**
  * @brief  Get SPI raw interrupt flags
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RAW_INTSTAT          | RAW_INTSTAT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_SSI_RIS_MST(no effect in SPIS)
  *         @arg @ref LL_SSI_RIS_RXF
  *         @arg @ref LL_SSI_RIS_RXO
  *         @arg @ref LL_SSI_RIS_RXU
  *         @arg @ref LL_SSI_RIS_TXO
  *         @arg @ref LL_SSI_RIS_TXE
  */
__STATIC_INLINE uint32_t ll_spi_get_raw_if_flag(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->RAW_INTSTAT));
}

/**
  * @brief  Clear transmit FIFO overflow error flag
  * @note   Clearing this flag is done by reading TXOIC register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TXOIC                | TXOIC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_txo(ssi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->TXOIC;
    (void) tmpreg;
}

/**
  * @brief  Clear receive FIFO overflow error flag
  * @note   Clearing this flag is done by reading RXOIC register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RXOIC                | RXOIC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_rxo(ssi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->RXOIC;
    (void) tmpreg;
}

/**
  * @brief  Clear receive FIFO underflow error flag
  * @note   Clearing this flag is done by reading RXUIC register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RXUIC                | RXUIC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_rxu(ssi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->RXUIC;
    (void) tmpreg;
}

/**
  * @brief  Clear multi-master error flag
  * @note   Clearing this flag is done by reading MSTIC register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSTIC                | MSTIC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_mst(ssi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->MSTIC;
    (void) tmpreg;
}

/**
  * @brief  Clear all error flag
  * @note   Clearing this flag is done by reading INTCLR register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTCLR               | INTCLR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_flag_all(ssi_regs_t *SPIx)
{
    __IOM uint32_t tmpreg;
    tmpreg = SPIx->INTCLR;
    (void) tmpreg;
}

/** @} */

/** @defgroup SPI_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA Tx
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMAC                 | TDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_dma_req_tx(ssi_regs_t *SPIx)
{
    SET_BITS(SPIx->DMAC, SSI_DMAC_TDMAE);
}

/**
  * @brief  Disable DMA Tx
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMAC                 | TDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_dma_req_tx(ssi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->DMAC, SSI_DMAC_TDMAE);
}

/**
  * @brief  Check if DMA Tx is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMAC                 | TDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_dma_req_tx(ssi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->DMAC, SSI_DMAC_TDMAE) == (SSI_DMAC_TDMAE));
}

/**
  * @brief  Enable DMA Rx
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMAC                 | RDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_enable_dma_req_rx(ssi_regs_t *SPIx)
{
    SET_BITS(SPIx->DMAC, SSI_DMAC_RDMAE);
}

/**
  * @brief  Disable DMA Rx
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMAC                 | RDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable_dma_req_rx(ssi_regs_t *SPIx)
{
    CLEAR_BITS(SPIx->DMAC, SSI_DMAC_RDMAE);
}

/**
  * @brief  Check if DMA Rx is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMAC                 | RDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_spi_is_enabled_dma_req_rx(ssi_regs_t *SPIx)
{
    return (READ_BITS(SPIx->DMAC, SSI_DMAC_RDMAE) == (SSI_DMAC_RDMAE));
}

/**
  * @brief  Set threshold of TXFIFO that triggers an DMA Tx request event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMATDL               | DMATDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_dma_tx_fifo_threshold(ssi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->DMA_TDL, threshold);
}

/**
  * @brief  Get threshold of TXFIFO that triggers an DMA Tx request event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMATDL               | DMATDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_spi_get_dma_tx_fifo_threshold(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->DMA_TDL, SSI_DMATDL_DMATDL));
}

/**
  * @brief  Set threshold of RXFIFO that triggers an DMA Rx request event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMARDL               | DMARDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  threshold This parameter can be one of the following values: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_dma_rx_fifo_threshold(ssi_regs_t *SPIx, uint32_t threshold)
{
    WRITE_REG(SPIx->DMA_RDL, threshold);
}

/**
  * @brief  Get threshold of RXFIFO that triggers an DMA Rx request event
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DMARDL               | DMARDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_spi_get_dma_rx_fifo_threshold(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->DMA_RDL, SSI_DMARDL_DMARDL));
}

/** @} */

/** @defgroup SPI_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Write 8-Bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DATA                 | DATA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  tx_data Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void ll_spi_transmit_data8(ssi_regs_t *SPIx, uint8_t tx_data)
{
    *((__IOM uint8_t *)&SPIx->DATA) = tx_data;
}

/**
  * @brief  Write 16-Bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DATA                 | DATA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  tx_data Value between Min_Data=0x0000 and Max_Data=0xFFFF
  * @retval None
  */
__STATIC_INLINE void ll_spi_transmit_data16(ssi_regs_t *SPIx, uint16_t tx_data)
{
    *((__IOM uint16_t *)&SPIx->DATA) = tx_data;
}

/**
  * @brief  Write 32-Bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DATA                 | DATA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  tx_data Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_spi_transmit_data32(ssi_regs_t *SPIx, uint32_t tx_data)
{
    *((__IOM uint32_t *)&SPIx->DATA) = tx_data;
}

/**
  * @brief  Read 8-Bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DATA                 | DATA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Rerturned Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t ll_spi_receive_data8(ssi_regs_t *SPIx)
{
    return (uint8_t)(READ_REG(SPIx->DATA));
}

/**
  * @brief  Read 16-Bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DATA                 | DATA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned Value between Min_Data=0x0000 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint16_t ll_spi_receive_data16(ssi_regs_t *SPIx)
{
    return (uint16_t)(READ_REG(SPIx->DATA));
}

/**
  * @brief  Read 32-Bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DATA                 | DATA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_spi_receive_data32(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->DATA));
}

/**
  * @brief  Set Rx sample delay
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RX_SAMPLEDLY         | RX_SAMPLEDLY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  delay This parameter can be one of the following values: 0 ~ 256
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_rx_sample_delay(ssi_regs_t *SPIx, uint32_t delay)
{
    WRITE_REG(SPIx->RX_SAMPLE_DLY, delay);
}

/**
  * @brief  Get Rx sample delay
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RX_SAMPLEDLY         | RX_SAMPLEDLY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 256
  */
__STATIC_INLINE uint32_t ll_spi_get_rx_sample_delay(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_REG(SPIx->RX_SAMPLE_DLY));
}

/**
  * @brief  Set number of wait cycles in Dual/Quad SPI mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | WAITCYCLES                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  wait_cycles This parameter can be one of the following values: 0 ~ 31
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_wait_cycles(ssi_regs_t *SPIx, uint32_t wait_cycles)
{
    MODIFY_REG(SPIx->SPI_CTRL0, SSI_SCTRL0_WAITCYCLES, wait_cycles << SSI_SCTRL0_WAITCYCLES_Pos);
}

/**
  * @brief  Get number of wait cycles in Dual/Quad SPI mode
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | WAITCYCLES                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values: 0 ~ 31
  */
__STATIC_INLINE uint32_t ll_spi_get_wait_cycles(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->SPI_CTRL0, SSI_SCTRL0_WAITCYCLES) >> SSI_SCTRL0_WAITCYCLES_Pos);
}

/**
  * @brief  Set Dual/Quad SPI mode instruction length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | INSTL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_SSI_INSTSIZE_0BIT
  *         @arg @ref LL_SSI_INSTSIZE_4BIT
  *         @arg @ref LL_SSI_INSTSIZE_8BIT
  *         @arg @ref LL_SSI_INSTSIZE_16BIT
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_instruction_size(ssi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->SPI_CTRL0, SSI_SCTRL0_INSTL, size);
}

/**
  * @brief  Get Dual/Quad SPI mode instruction length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | INSTL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_INSTSIZE_0BIT
  *         @arg @ref LL_SSI_INSTSIZE_4BIT
  *         @arg @ref LL_SSI_INSTSIZE_8BIT
  *         @arg @ref LL_SSI_INSTSIZE_16BIT
  */
__STATIC_INLINE uint32_t ll_spi_get_instruction_size(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->SPI_CTRL0, SSI_SCTRL0_INSTL));
}

/**
  * @brief  Set Dual/Quad SPI mode address length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | ADDRL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  size This parameter can be one of the following values:
  *         @arg @ref LL_SSI_ADDRSIZE_0BIT
  *         @arg @ref LL_SSI_ADDRSIZE_4BIT
  *         @arg @ref LL_SSI_ADDRSIZE_8BIT
  *         @arg @ref LL_SSI_ADDRSIZE_12BIT
  *         @arg @ref LL_SSI_ADDRSIZE_16BIT
  *         @arg @ref LL_SSI_ADDRSIZE_20BIT
  *         @arg @ref LL_SSI_ADDRSIZE_24BIT
  *         @arg @ref LL_SSI_ADDRSIZE_28BIT
  *         @arg @ref LL_SSI_ADDRSIZE_32BIT
  *         @arg @ref LL_SSI_ADDRSIZE_36BIT
  *         @arg @ref LL_SSI_ADDRSIZE_40BIT
  *         @arg @ref LL_SSI_ADDRSIZE_44BIT
  *         @arg @ref LL_SSI_ADDRSIZE_48BIT
  *         @arg @ref LL_SSI_ADDRSIZE_52BIT
  *         @arg @ref LL_SSI_ADDRSIZE_56BIT
  *         @arg @ref LL_SSI_ADDRSIZE_60BIT
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_address_size(ssi_regs_t *SPIx, uint32_t size)
{
    MODIFY_REG(SPIx->SPI_CTRL0, SSI_SCTRL0_ADDRL, size);
}

/**
  * @brief  Get Dual/Quad SPI mode address length in bits
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | ADDRL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_ADDRSIZE_0BIT
  *         @arg @ref LL_SSI_ADDRSIZE_4BIT
  *         @arg @ref LL_SSI_ADDRSIZE_8BIT
  *         @arg @ref LL_SSI_ADDRSIZE_12BIT
  *         @arg @ref LL_SSI_ADDRSIZE_16BIT
  *         @arg @ref LL_SSI_ADDRSIZE_20BIT
  *         @arg @ref LL_SSI_ADDRSIZE_24BIT
  *         @arg @ref LL_SSI_ADDRSIZE_28BIT
  *         @arg @ref LL_SSI_ADDRSIZE_32BIT
  *         @arg @ref LL_SSI_ADDRSIZE_36BIT
  *         @arg @ref LL_SSI_ADDRSIZE_40BIT
  *         @arg @ref LL_SSI_ADDRSIZE_44BIT
  *         @arg @ref LL_SSI_ADDRSIZE_48BIT
  *         @arg @ref LL_SSI_ADDRSIZE_52BIT
  *         @arg @ref LL_SSI_ADDRSIZE_56BIT
  *         @arg @ref LL_SSI_ADDRSIZE_60BIT
  */
__STATIC_INLINE uint32_t ll_spi_get_address_size(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->SPI_CTRL0, SSI_SCTRL0_ADDRL));
}

/**
  * @brief  Set Dual/Quad SPI mode address and instruction transfer format
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | TRANSTYPE                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @param  format This parameter can be one of the following values:
  *         @arg @ref LL_SSI_INST_ADDR_ALL_IN_SPI
  *         @arg @ref LL_SSI_INST_IN_SPI_ADDR_IN_SPIFRF
  *         @arg @ref LL_SSI_INST_ADDR_ALL_IN_SPIFRF
  * @retval None
  */
__STATIC_INLINE void ll_spi_set_add_inst_transfer_format(ssi_regs_t *SPIx, uint32_t format)
{
    MODIFY_REG(SPIx->SPI_CTRL0, SSI_SCTRL0_TRANSTYPE, format);
}

/**
  * @brief  Get Dual/Quad SPI mode address and instruction transfer format
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SCTRL0               | TRANSTYPE                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  SPIx SPI instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SSI_INST_ADDR_ALL_IN_SPI
  *         @arg @ref LL_SSI_INST_IN_SPI_ADDR_IN_SPIFRF
  *         @arg @ref LL_SSI_INST_ADDR_ALL_IN_SPIFRF
  */
__STATIC_INLINE uint32_t ll_spi_get_addr_inst_transfer_format(ssi_regs_t *SPIx)
{
    return (uint32_t)(READ_BITS(SPIx->SPI_CTRL0, SSI_SCTRL0_TRANSTYPE));
}

/** @} */

/** @defgroup SPI_LL_EF_Init SPIM Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize SSI registers (Registers restored to their default values).
  * @param  SPIx SSI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are de-initialized
  *          - ERROR: SSI registers are not de-initialized
  */
error_status_t ll_spim_deinit(ssi_regs_t *SPIx);

/**
  * @brief  Initialize SPIM registers according to the specified
  *         parameters in p_spi_init.
  * @param  SPIx SSI instance
  * @param  p_spi_init Pointer to a ll_spim_init_t structure that contains the configuration
  *                         information for the specified SPIM peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are initialized according to p_spi_init content
  *          - ERROR: Problem occurred during SSI Registers initialization
  */
error_status_t ll_spim_init(ssi_regs_t *SPIx, ll_spim_init_t *p_spi_init);

/**
  * @brief Set each field of a @ref ll_spim_init_t type structure to default value.
  * @param p_spi_init  Pointer to a @ref ll_spim_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_spim_struct_init(ll_spim_init_t *p_spi_init);

/** @} */

/** @defgroup SPIS_LL_Init SPIS Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize SSI registers (Registers restored to their default values).
  * @param  SPIx SSI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are de-initialized
  *          - ERROR: SSI registers are not de-initialized
  */
error_status_t ll_spis_deinit(ssi_regs_t *SPIx);

/**
  * @brief  Initialize SSI registers according to the specified
  *         parameters in p_spi_init.
  * @param  SPIx SSI instance
  * @param  p_spi_init Pointer to a ll_spis_init_t structure that contains the configuration
  *                         information for the specified SPIS peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are initialized according to p_spi_init content
  *          - ERROR: Problem occurred during SPI Registers initialization
  */
error_status_t ll_spis_init(ssi_regs_t *SPIx, ll_spis_init_t *p_spi_init);

/**
  * @brief Set each field of a @ref ll_spis_init_t type structure to default value.
  * @param p_spi_init  Pointer to a @ref ll_spis_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_spis_struct_init(ll_spis_init_t *p_spi_init);
/** @} */

/** @defgroup QSPI_LL_Init QSPI Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize SSI registers (Registers restored to their default values).
  * @param  SPIx SSI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SSI registers are de-initialized
  *          - ERROR: SSI registers are not de-initialized
  */
error_status_t ll_qspi_deinit(ssi_regs_t *SPIx);

/**
  * @brief  Initialize SSI registers according to the specified
  *         parameters in SPI_InitStruct.
  * @param  SPIx SSI instance
  * @param  p_spi_init Pointer to a ll_qspi_init_t structure that contains the configuration
  *                         information for the specified QSPI peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: SPI registers are initialized according to p_spi_init content
  *          - ERROR: Problem occurred during SPI Registers initialization
  */
error_status_t ll_qspi_init(ssi_regs_t *SPIx, ll_qspi_init_t *p_spi_init);

/**
  * @brief Set each field of a @ref ll_qspi_init_t type structure to default value.
  * @param p_spi_init  Pointer to a @ref ll_qspi_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_qspi_struct_init(ll_qspi_init_t *p_spi_init);

/** @} */

/** @} */

#endif /* SPIM || SPIS || QSPI0 || QSPI1 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_SPI_H__ */

/** @} */

/** @} */

/** @} */
