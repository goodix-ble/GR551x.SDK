/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_i2c.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2C LL library.
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

/** @defgroup LL_I2C I2C
  * @brief I2C LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_I2C_H__
#define __GR55xx_LL_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (I2C0) || defined (I2C1)

/** @defgroup I2C_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2C_LL_ES_INIT I2C Exported init structure
  * @{
  */

/**
  * @brief LL I2C init Structure definition
  */
typedef struct _ll_i2c_init
{
    uint32_t speed;               /**< Specifies the transfer speed. See @ref I2C_LL_EC_SPEED. */

    uint32_t own_address;         /**< Specifies the device own address.
                                     This parameter must be a value between Min_Data = 0x00 and Max_Data = 0x3FF

                                     This feature can be modified afterwards using unitary function @ref ll_i2c_set_own_address(). */

    uint32_t own_addr_size;       /**< Specifies the device own address 1 size (7-bit or 10-bit).
                                     This parameter can be a value of @ref I2C_LL_EC_OWNADDRESS

                                     This feature can be modified afterwards using unitary function @ref ll_i2c_set_own_address(). */
} ll_i2c_init_t;

/** @} */

/** @} */

/**
  * @defgroup  I2C_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2C_LL_Exported_Constants I2C Exported Constants
  * @{
  */

/** @defgroup I2C_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_I2C_ReadReg function
  * @{
  */
#define LL_I2C_INTR_STAT_MST_ON_HOLD        I2C_INTR_GEN_CALL       /**< MST_ON_HOLD interrupt flag */
#define LL_I2C_INTR_STAT_RESTART_DET        I2C_INTR_RESTART_DET    /**< RESTART_DET interrupt flag */
#define LL_I2C_INTR_STAT_GEN_CALL           I2C_INTR_GEN_CALL       /**< GEN_CALL interrupt flag */
#define LL_I2C_INTR_STAT_START_DET          I2C_INTR_START_DET      /**< START_DET interrupt flag */
#define LL_I2C_INTR_STAT_STOP_DET           I2C_INTR_STOP_DET       /**< STOP_DET interrupt flag */
#define LL_I2C_INTR_STAT_ACTIVITY           I2C_INTR_ACTIVITY       /**< ACTIVITY interrupt flag */
#define LL_I2C_INTR_STAT_RX_DONE            I2C_INTR_RX_DONE        /**< RX_DONE interrupt flag */
#define LL_I2C_INTR_STAT_TX_ABRT            I2C_INTR_TX_ABRT        /**< TX_ABRT interrupt flag */
#define LL_I2C_INTR_STAT_RD_REQ             I2C_INTR_RD_REQ         /**< RD_REQ interrupt flag */
#define LL_I2C_INTR_STAT_TX_EMPTY           I2C_INTR_TX_EMPTY       /**< TX_EMPTY interrupt flag */
#define LL_I2C_INTR_STAT_TX_OVER            I2C_INTR_TX_OVER        /**< TX_OVER interrupt flag */
#define LL_I2C_INTR_STAT_RX_FULL            I2C_INTR_RX_FULL        /**< RX_FULL interrupt flag */
#define LL_I2C_INTR_STAT_RX_OVER            I2C_INTR_RX_OVER        /**< RX_OVER interrupt flag */
#define LL_I2C_INTR_STAT_RX_UNDER           I2C_INTR_RX_UNDER       /**< RX_UNDER interrupt flag */

#define LL_I2C_ABRT_TX_FLUSH_CNT            I2C_TX_ABRT_SRC_TX_FLUSH_CNT    /**< Transfer abort detected by master */
#define LL_I2C_ABRT_USER_ABRT               I2C_TX_ABRT_SRC_USER_ABRT       /**< Transfer abort detected by master */
#define LL_I2C_ABRT_SLVRD_INTX              I2C_TX_ABRT_SRC_SLVRD_INTX      /**< Slave trying to transmit to remote master in read mode */
#define LL_I2C_ABRT_SLV_ARBLOST             I2C_TX_ABRT_SRC_SLV_ARBLOST     /**< Slave lost arbitration to remote master */
#define LL_I2C_ABRT_SLVFLUSH_TXFIFO         I2C_TX_ABRT_SRC_SLVFLUSH_TXFIFO /**< Slave flushes existing data in TX-FIFO upon getting read command */
#define LL_I2C_ABRT_ARB_LOST                I2C_TX_ABRT_SRC_ARB_LOST        /**< Master or Slave Transmitter lost arbitration */
#define LL_I2C_ABRT_MST_DIS                 I2C_TX_ABRT_SRC_MST_DIS         /**< User intitating master operation when MASTER disabled */
#define LL_I2C_ABRT_10B_RD_NORSTRT          I2C_TX_ABRT_SRC_10B_RD_NORSTRT  /**< Master trying to read in 10-Bit addressing mode when RESTART disabled */
#define LL_I2C_ABRT_SBYTE_NORSTRT           I2C_TX_ABRT_SRC_SBYTE_NORSTRT   /**< User trying to send START byte when RESTART disabled */
#define LL_I2C_ABRT_HS_NORSTRT              I2C_TX_ABRT_SRC_HS_NORSTRT      /**< User trying to swidth Master to HS mode when RESTART disabled */
#define LL_I2C_ABRT_SBYTE_ACKDET            I2C_TX_ABRT_SRC_SBYTE_ACKDET    /**< ACK detected for START byte */
#define LL_I2C_ABRT_HS_ACKDET               I2C_TX_ABRT_SRC_HS_ACKDET       /**< HS Master code is ACKed in HS Mode */
#define LL_I2C_ABRT_GCALL_READ              I2C_TX_ABRT_SRC_GCALL_READ      /**< GCALL is followed by read from bus */
#define LL_I2C_ABRT_GCALL_NOACK             I2C_TX_ABRT_SRC_GCALL_NOACK     /**< GCALL is not ACKed by any slave */
#define LL_I2C_ABRT_TXDATA_NOACK            I2C_TX_ABRT_SRC_TXDATA_NOACK    /**< Transmitted data is not ACKed by addressed slave */
#define LL_I2C_ABRT_10ADDR2_NOACK           I2C_TX_ABRT_SRC_10ADDR2_NOACK   /**< Byte 2 of 10-Bit Address is not ACKed by any slave */
#define LL_I2C_ABRT_10ADDR1_NOACK           I2C_TX_ABRT_SRC_10ADDR1_NOACK   /**< Byte 1 of 10-Bit Address is not ACKed by any slave */
#define LL_I2C_ABRT_7B_ADDR_NOACK           I2C_TX_ABRT_SRC_7B_ADDR_NOACK   /**< 7Bit Address is not ACKed by any slave */
/** @} */

/** @defgroup I2C_LL_EC_IT IT Defines
  * @brief    Interrupt definitions which can be used with LL_I2C_ReadReg and  LL_I2C_WriteReg functions
  * @{
  */
#define LL_I2C_INTR_MASK_MST_ON_HOLD        I2C_INTR_GEN_CALL       /**< MST_ON_HOLD interrupt */
#define LL_I2C_INTR_MASK_RESTART_DET        I2C_INTR_RESTART_DET    /**< RESTART_DET interrupt */
#define LL_I2C_INTR_MASK_GEN_CALL           I2C_INTR_GEN_CALL       /**< GEN_CALL interrupt */
#define LL_I2C_INTR_MASK_START_DET          I2C_INTR_START_DET      /**< START_DET interrupt */
#define LL_I2C_INTR_MASK_STOP_DET           I2C_INTR_STOP_DET       /**< STOP_DET interrupt */
#define LL_I2C_INTR_MASK_ACTIVITY           I2C_INTR_ACTIVITY       /**< ACTIVITY interrupt */
#define LL_I2C_INTR_MASK_RX_DONE            I2C_INTR_RX_DONE        /**< RX_DONE interrupt */
#define LL_I2C_INTR_MASK_TX_ABRT            I2C_INTR_TX_ABRT        /**< TX_ABRT interrupt */
#define LL_I2C_INTR_MASK_RD_REQ             I2C_INTR_RD_REQ         /**< RD_REQ interrupt */
#define LL_I2C_INTR_MASK_TX_EMPTY           I2C_INTR_TX_EMPTY       /**< TX_EMPTY interrupt */
#define LL_I2C_INTR_MASK_TX_OVER            I2C_INTR_TX_OVER        /**< TX_OVER interrupt */
#define LL_I2C_INTR_MASK_RX_FULL            I2C_INTR_RX_FULL        /**< RX_FULL interrupt */
#define LL_I2C_INTR_MASK_RX_OVER            I2C_INTR_RX_OVER        /**< RX_OVER interrupt */
#define LL_I2C_INTR_MASK_RX_UNDER           I2C_INTR_RX_UNDER       /**< RX_UNDER interrupt */

#define LL_I2C_INTR_MASK_ALL                0x00000FFFU             /**< All interrupt */
/** @} */

/** @defgroup I2C_LL_EC_ADDRESSING_MODE Master Addressing Mode
  * @{
  */
#define LL_I2C_ADDRESSING_MODE_7BIT         0x00000000U             /**< Master operates in 7-bit addressing mode. */
#define LL_I2C_ADDRESSING_MODE_10BIT        I2C_CON_10BITADDR_MST   /**< Master operates in 10-bit addressing mode.*/
/** @} */

/** @defgroup I2C_LL_EC_OWNADDRESS Own Address Length
  * @{
  */
#define LL_I2C_OWNADDRESS_7BIT              0x00000000U             /**< Own address 1 is a 7-bit address. */
#define LL_I2C_OWNADDRESS_10BIT             I2C_CON_10BITADDR_SLV   /**< Own address 1 is a 10-bit address.*/
/** @} */


/** @defgroup I2C_LL_EC_GENERATE Start And Stop Generation
  * @{
  */
#define LL_I2C_CMD_SLV_NONE                 0x00000000U             /**< Slave No command. */
#define LL_I2C_CMD_MST_WRITE                0x00000000U             /**< Master write command. */
#define LL_I2C_CMD_MST_READ                 I2C_DATA_CMD_CMD        /**< Master read command.  */
#define LL_I2C_CMD_MST_GEN_STOP             I2C_DATA_CMD_STOP       /**< Master issue STOP after this command.  */
#define LL_I2C_CMD_MST_GEN_RESTART          I2C_DATA_CMD_RESTART    /**< Master issue RESTART before this command.  */
/** @} */

/** @defgroup I2C_LL_EC_SPEED_MODE Transfer Speed Mode
  * @{
  */
#define LL_I2C_SPEED_MODE_STANDARD          I2C_CON_SPEED_STANDARD  /**< Standard Speed mode(0 to 100 Kb/s) of operation. */
#define LL_I2C_SPEED_MODE_FAST              I2C_CON_SPEED_FAST      /**< Fast (鈮?400 Kb/s) or Fast Plus mode (鈮?1000 螝b/s) of operation. */
#define LL_I2C_SPEED_MODE_HIGH              I2C_CON_SPEED_HIGH      /**< High Speed mode (鈮?3.4 Mb/s) of operation. */
/** @} */

/** @defgroup I2C_LL_EC_SPEED Transfer Speed
  * @{
  */
#define LL_I2C_SPEED_100K                   (100000ul)              /**< Standard Speed.  */
#define LL_I2C_SPEED_400K                   (400000ul)              /**< Fast Speed.      */
#define LL_I2C_SPEED_1000K                  (1000000ul)             /**< Fast Plus Speed. */
#define LL_I2C_SPEED_2000K                  (2000000ul)             /**< High Speed.      */
/** @} */

/** @defgroup I2C_LL_EC_DIRECTION Read Write Direction
  * @{
  */
#define LL_I2C_DIRECTION_NONE               0x00000000U                         /**< No transfer request by master. */
#define LL_I2C_DIRECTION_WRITE              I2C_INTR_RX_FULL                    /**< Write transfer request by master, slave enters receiver mode.  */
#define LL_I2C_DIRECTION_READ               I2C_INTR_RD_REQ                     /**< Read transfer request by master, slave enters transmitter mode.*/
#define LL_I2C_DIRECTION_ERROR              I2C_INTR_RX_FULL | I2C_INTR_RD_REQ  /**< Transfer request error. */
/** @} */


/** @defgroup I2C_LL_EC_TX_FIFO_TH TX FIFO Threshold
  * @{
  */
#define LL_I2C_TX_FIFO_TH_EMPTY             0x00000000U     /**< TX FIFO empty */
#define LL_I2C_TX_FIFO_TH_CHAR_1            0x00000001U     /**< 1 character in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_2            0x00000002U     /**< 2 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_3            0x00000003U     /**< 3 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_4            0x00000004U     /**< 4 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_5            0x00000005U     /**< 5 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_6            0x00000006U     /**< 6 characters in TX FIFO */
#define LL_I2C_TX_FIFO_TH_CHAR_7            0x00000007U     /**< 7 characters in TX FIFO */
/** @} */

/** @defgroup I2C_LL_EC_RX_FIFO_TH RX FIFO Threshold
  * @{
  */
#define LL_I2C_RX_FIFO_TH_CHAR_1            0x00000000U     /**< 1 character in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_2            0x00000001U     /**< 2 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_3            0x00000002U     /**< 3 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_4            0x00000003U     /**< 4 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_5            0x00000004U     /**< 5 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_6            0x00000005U     /**< 6 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_CHAR_7            0x00000006U     /**< 7 characters in RX FIFO */
#define LL_I2C_RX_FIFO_TH_FULL              0x00000007U     /**< RX FIFO full */
/** @} */

/** @defgroup I2C_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL I2C InitStrcut default configuartion
  */
#define LL_I2C_DEFAULT_CONFIG                          \
{                                                      \
    .speed           = LL_I2C_SPEED_400K,              \
    .own_address     = 0x55U,                          \
    .own_addr_size   = LL_I2C_OWNADDRESS_7BIT,         \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I2C_LL_Exported_Macros I2C Exported Macros
  * @{
  */

/** @defgroup I2C_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in I2C register
  * @param  __instance__ I2C instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None.
  */
#define LL_I2C_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in I2C register
  * @param  __instance__ I2C instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_I2C_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)
/** @} */

/** @defgroup I2C_LL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Compute CLK_SSL_CNT value according to Peripheral Clock and expected Speed.
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for I2C instance
  * @param  __SPEED__ Speed value to achieve
  * @retval CLK_SSL_CNT value to be used for XS_SCL_HCNT, XS_SCL_LCNT registers where X can be (S, F, H)
  */
#define __LL_I2C_CONVERT_CLK_SSL_CNT(__PERIPHCLK__, __SPEED__) ((__PERIPHCLK__) / 2 / (__SPEED__))

/**
  * @brief  Get Speed Mode according to expected Speed.
  * @param  __SPEED__ Speed value to achieve
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_SPEED_MODE_STANDARD
  *         @arg @ref LL_I2C_SPEED_MODE_FAST
  *         @arg @ref LL_I2C_SPEED_MODE_HIGH
  */
#define __LL_I2C_CONVERT_SPEED_MODE(__SPEED__)      ((__SPEED__ <= LL_I2C_SPEED_100K) ? LL_I2C_SPEED_MODE_STANDARD : \
                                                     ((__SPEED__ <= LL_I2C_SPEED_1000K) ? LL_I2C_SPEED_MODE_FAST : LL_I2C_SPEED_MODE_HIGH))
/** @} */

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup I2C_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup I2C_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Enable I2C peripheral (ENABLE = 1).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ENABLE            | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->ENABLE, I2C_ENABLE_ENABLE);
}

/**
  * @brief  Disable I2C peripheral (ENABLE = 0).
  * @note   When ENABLE = 0, the TX FIFO and RX FIFO get flushed.
  *         Status bits in the IC_INTR_STAT register are still active until DW_apb_i2c goes into IDLE state.
  *         If the module is transmitting, it stops as well as deletes the contents of the transmit buffer
  *         after the current transfer is complete. If the module is receiving, the DW_apb_i2c stops
  *         the current transfer at the end of the current byte and does not acknowledge the transfer..
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ENABLE            | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->ENABLE, I2C_ENABLE_ENABLE);
}

/**
  * @brief  Check if the I2C peripheral is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ENABLE_STATUS     | IC_EN                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->ENABLE_STATUS, I2C_ENABLE_STATUS_IC_EN) == (I2C_ENABLE_STATUS_IC_EN));
}

/**
  * @brief  Enable I2C master mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | MASTER_ENABLE                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_CON | SLAVE_DISABLE
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_master_mode(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CON, I2C_CON_MST_MODE | I2C_CON_SLV_DIS);
}

/**
  * @brief  Disable I2C master mode and enable slave mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | MASTER_ENABLE                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_CON | SLAVE_DISABLE
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_master_mode(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CON, I2C_CON_MST_MODE | I2C_CON_SLV_DIS);
}

/**
  * @brief  Check if I2C master mode is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | MASTER_ENABLE                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_CON | SLAVE_DISABLE
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_master_mode(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->CON, I2C_CON_MST_MODE | I2C_CON_SLV_DIS) == (I2C_CON_MST_MODE | I2C_CON_SLV_DIS));
}

/**
  * @brief  Enable General Call(slave mode).
  * @note   When enabled, the Address 0x00 is ACKed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ACK_GENERAL_CALL  | ACK_GEN_CALL                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_general_call(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->ACK_GENERAL_CALL, I2C_ACK_GENERAL_CALL_ACK_GC);
}

/**
  * @brief  Disable General Call(slave mode).
  * @note   When disabled, the Address 0x00 is NACKed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ACK_GENERAL_CALL  | ACK_GEN_CALL                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_general_call(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->ACK_GENERAL_CALL, I2C_ACK_GENERAL_CALL_ACK_GC);
}

/**
  * @brief  Check if General Call is enabled or disabled(slave mode).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ACK_GENERAL_CALL  | ACK_GEN_CALL                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_general_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->ACK_GENERAL_CALL, I2C_ACK_GENERAL_CALL_ACK_GC) == (I2C_ACK_GENERAL_CALL_ACK_GC));
}

/**
  * @brief  Enable Master Restart.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *         This bit determines whether RESTART conditions may be sent when acting as a master.
  *         Some older slaves do not support handling RESTART conditions.
  *         When RESTART is disabled, the master is prohibited from performing the following functions:
  *         - Performing any high-speed mode operation.
  *         - Performing direction changes in combined format mode.
  *         - Performing a read operation with a 10-bit address.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | CON_RESTART_EN                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_master_restart(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CON, I2C_CON_RESTART_EN);
}

/**
  * @brief  Disable Master Restart.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | CON_RESTART_EN                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_master_restart(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CON, I2C_CON_RESTART_EN);
}

/**
  * @brief  Check if Master Restart is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | CON_RESTART_EN                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_master_restart(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->CON, I2C_CON_RESTART_EN) == (I2C_CON_RESTART_EN));
}

/**
  * @brief  Enable Slave issues STOP_DET interrupt only if addressed function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *         During a general call address, the slave does not issue the STOP_DET interrupt if
  *         STOP_DET_IF_ADDRESSED = 1'b1, even if the slave responds to the general call address
  *         by generating ACK. The STOP_DET interrupt is generated only when the transmitted
  *         address matches the slave address (SAR).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | STOP_DET_IF_ADDRESSED             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_stop_det_if_addressed(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->CON, I2C_CON_STOP_DET_IF_ADDRESSED);
}

/**
  * @brief  Disable Slave issues STOP_DET interrupt only if addressed function.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | STOP_DET_IF_ADDRESSED             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_stop_det_if_addressed(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->CON, I2C_CON_STOP_DET_IF_ADDRESSED);
}

/**
  * @brief  Check if Slave issues STOP_DET interrupt only if addressed function is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | STOP_DET_IF_ADDRESSED             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_stop_det_if_addressed(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->CON, I2C_CON_STOP_DET_IF_ADDRESSED) == (I2C_CON_STOP_DET_IF_ADDRESSED));
}

/**
  * @brief  Configure the Master to transfers in 7-bit or 10-bit addressing mode.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | CON_10BITADDR_MST                 |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  addressing_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_master_addressing_mode(i2c_regs_t *I2Cx, uint32_t addressing_mode)
{
    MODIFY_REG(I2Cx->CON, I2C_CON_10BITADDR_MST, addressing_mode);
}

/**
  * @brief  Get the Master addressing mode.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | CON_10BITADDR_MST                 |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  */
__STATIC_INLINE uint32_t ll_i2c_get_master_addressing_mode(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->CON, I2C_CON_10BITADDR_MST));
}

/**
  * @brief  Set the Own Address.
  * @note   The register IC_CON and IC_SAR can only be programmed when the I2C is disabled (IC_ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | CON_10BITADDR_SLV                 |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_SAR | SAR
  *
  * @param  I2Cx I2C instance.
  * @param  own_address  This parameter must be a value range between 0 ~ 0x3FF(10-bit mode) or 0 ~ 0x7F(7-bit mode).
  *         Reserved address 0x00 to 0x07, or 0x78 to 0x7f should not be configured.
  * @param  own_addr_size This parameter can be one of the following values:
  *         @arg @ref LL_I2C_OWNADDRESS_7BIT
  *         @arg @ref LL_I2C_OWNADDRESS_10BIT
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_own_address(i2c_regs_t *I2Cx, uint32_t own_address, uint32_t own_addr_size)
{
    MODIFY_REG(I2Cx->CON, I2C_CON_10BITADDR_SLV, own_addr_size);
    WRITE_REG(I2Cx->SAR, own_address);
}

/**
  * @brief  Set the SCL clock high-period count for standard speed.
  * @note   The register IC_SS_SCL_HCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SS_SCL_HCNT       | SS_SCL_HCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  count This parameter must be a value range between 6 ~ 0xFFF5.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_high_period_ss(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->SS_SCL_HCNT, count);
}

/**
  * @brief  Get the SCL clock high-period count for standard speed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SS_SCL_HCNT       | SS_SCL_HCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x6 and 0xFFF5.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_high_period_ss(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SS_SCL_HCNT, I2C_SS_SCL_HCNT));
}

/**
  * @brief  Set the SCL clock low-period count for standard speed.
  * @note   The register IC_SS_SCL_LCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SS_SCL_LCNT       | SS_SCL_LCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  count This parameter must be a value range between 0x8 and 0xFFFF.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_low_period_ss(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->SS_SCL_LCNT, count);
}

/**
  * @brief  Get the SCL clock low-period count for standard speed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SS_SCL_LCNT       | SS_SCL_LCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x8 and 0xFFFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_low_period_ss(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SS_SCL_LCNT, I2C_SS_SCL_LCNT));
}

/**
  * @brief  Set the SCL clock high-period count for fast speed.
  * @note   The register IC_FS_SCL_HCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_FS_SCL_HCNT       | FS_SCL_HCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x6 and 0xFFFF.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_high_period_fs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->FS_SCL_HCNT, count);
}

/**
  * @brief  Get the SCL clock high-period count for fast speed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_FS_SCL_HCNT       | FS_SCL_HCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x6 and 0xFFFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_high_period_fs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->FS_SCL_HCNT, I2C_FS_SCL_HCNT));
}

/**
  * @brief  Set the SCL clock low-period count for fast speed.
  * @note   The register IC_FS_SCL_LCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_FS_SCL_LCNT       | FS_SCL_LCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x8 and 0xFFFF
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_low_period_fs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->FS_SCL_LCNT, count);
}

/**
  * @brief  Get the SCL clock low-period count for fast speed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_FS_SCL_LCNT       | FS_SCL_LCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x8 and 0xFFFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_low_period_fs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->FS_SCL_LCNT, I2C_FS_SCL_LCNT));
}

/**
  * @brief  Get the SCL clock high-period count for high speed.
  * @note   The register IC_HS_SCL_HCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_SCL_HCNT       | HS_SCL_HCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x6 and 0xFFFF, should be larger than IC_HS_SPKLEN + 5.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_high_period_hs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->HS_SCL_HCNT, count);
}

/**
  * @brief  Get the SCL clock high-period count for high speed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_SCL_HCNT       | HS_SCL_HCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval range between 0x6 and 0xFFFF, should be larger than IC_HS_SPKLEN + 7.
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_high_period_hs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->HS_SCL_HCNT, I2C_HS_SCL_HCNT));
}

/**
  * @brief  Get the SCL clock low-period count for high speed.
  * @note   The register IC_HS_SCL_LCNT can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_SCL_LCNT       | HS_SCL_LCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  count range between 0x8 and 0xFFFF
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_clock_low_period_hs(i2c_regs_t *I2Cx, uint32_t count)
{
    WRITE_REG(I2Cx->HS_SCL_LCNT, count);
}

/**
  * @brief  Get the SCL clock low-period count for high speed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_SCL_LCNT       | HS_SCL_LCNT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x8 and 0xFFFF
  */
__STATIC_INLINE uint32_t ll_i2c_get_clock_low_period_hs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->HS_SCL_LCNT, I2C_HS_SCL_LCNT));
}

/**
  * @brief  Set the spike len in fast speed mode.
  * @note   The register FS_SPKLEN can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_FS_SPKLEN         | FS_SPKLEN                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  length  Spike len.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_spike_len_fs(i2c_regs_t *I2Cx, uint32_t length)
{
    MODIFY_REG(I2Cx->FS_SPKLEN, I2C_FS_SPKLEN_FS_SPKLEN, length);
}

/**
  * @brief  Get the spike len in fast speed mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_FS_SPKLEN         | FS_SPKLEN                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x2 and 0xFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_spike_len_fs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->FS_SPKLEN, I2C_FS_SPKLEN_FS_SPKLEN));
}

/**
  * @brief  Set the spike len in high speed mode.
  * @note   The register FS_SPKLEN can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_SPKLEN         | HS_SPKLEN                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  length  Spike len.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_spike_len_hs(i2c_regs_t *I2Cx, uint32_t length)
{
    MODIFY_REG(I2Cx->HS_SPKLEN, I2C_HS_SPKLEN_HS_SPKLEN, length);
}

/**
  * @brief  Get the spike len in high speed mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_SPKLEN         | HS_SPKLEN                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x2 and 0xFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_spike_len_hs(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->HS_SPKLEN, I2C_HS_SPKLEN_HS_SPKLEN));
}

/**
  * @brief  Set I2C Speed mode.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | SPEED                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  speed_mode This parameter can be one of the following values:
  *         @arg @ref LL_I2C_SPEED_MODE_STANDARD
  *         @arg @ref LL_I2C_SPEED_MODE_FAST
  *         @arg @ref LL_I2C_SPEED_MODE_HIGH
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_speed_mode(i2c_regs_t *I2Cx, uint32_t speed_mode)
{
    MODIFY_REG(I2Cx->CON, I2C_CON_SPEED, speed_mode);
}

/**
  * @brief  Get I2C Speed mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | SPEED                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value can be one of the following values:
  *         @arg @ref LL_I2C_SPEED_MODE_STANDARD
  *         @arg @ref LL_I2C_SPEED_MODE_FAST
  *         @arg @ref LL_I2C_SPEED_MODE_HIGH
  */
__STATIC_INLINE uint32_t ll_i2c_get_speed_mode(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->CON, I2C_CON_SPEED));
}

/**
  * @brief  Set I2C High Speed Master Code Address.
  * @note   The register IC_CON can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_MADDR          | HS_MAR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  code HS mode master code, range between 0x00 and 0x07.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_high_speed_master_code(i2c_regs_t *I2Cx, uint32_t code)
{
    WRITE_REG(I2Cx->HS_MADDR, code);
}

/**
  * @brief  Get I2C Speed mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_HS_MADDR          | HS_MAR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value range between 0x00 and 0x07.
  */
__STATIC_INLINE uint32_t ll_i2c_get_high_speed_master_code(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->HS_MADDR, I2C_HS_MADDR_HS_MAR));
}

/**
  * @brief  Set the required transmit SDA hold time in units of ic_clk period.
  * @note   The register IC_SDA_HOLD can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SDA_HOLD          | TX_HOLD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  time SDA Tx hold time in units of ic_clk period.
  *         Time should range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_data_tx_hold_time(i2c_regs_t *I2Cx, uint32_t time)
{
    MODIFY_REG(I2Cx->SDA_HOLD, I2C_SDA_HOLD_TX_HOLD, time << I2C_SDA_HOLD_TX_HOLD_Pos);
}

/**
  * @brief  Get the required transmit SDA hold time in units of ic_clk period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SDA_HOLD          | TX_HOLD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode
  */
__STATIC_INLINE uint32_t ll_i2c_get_data_tx_hold_time(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SDA_HOLD, I2C_SDA_HOLD_TX_HOLD) >> I2C_SDA_HOLD_TX_HOLD_Pos);
}

/**
  * @brief  Set the required receive SDA hold time in units of ic_clk period.
  * @note   The register IC_SDA_HOLD can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SDA_HOLD          | RX_HOLD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  time SDA Tx hold time in units of ic_clk period.
  *         Time should range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode.
  * @retval Value between Min_Data=0x0 and Max_Data=0xF
  */
__STATIC_INLINE void ll_i2c_set_data_rx_hold_time(i2c_regs_t *I2Cx, uint32_t time)
{
    MODIFY_REG(I2Cx->SDA_HOLD, I2C_SDA_HOLD_RX_HOLD, time << I2C_SDA_HOLD_RX_HOLD_Pos);
}

/**
  * @brief  Get the required receive SDA hold time in units of ic_clk period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SDA_HOLD          | RX_HOLD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 1 and (N_SCL_LOW - 2) in master mode or 7 and (N_SCL_LOW - 2) in slave mode
  */
__STATIC_INLINE uint32_t ll_i2c_get_data_rx_hold_time(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SDA_HOLD, I2C_SDA_HOLD_RX_HOLD) >> I2C_SDA_HOLD_RX_HOLD_Pos);
}

/**
  * @brief  Set the SDA setup time when operating as a slave transmitter.
  * @note   The register IC_SDA_SETUP can only be programmed when the I2C is disabled (ENABLE = 0).
  *         The length of setup time is calculated using [(IC_SDA_SETUP - 1) * (ic_clk_period)], so if the
  *         user requires 10 ic_clk periods of setup time, they should program a value of 11.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SDA_SETUP         | SDA_SETUP                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  time SDA data setup time in units of ic_clk period, range between 2 ~ 0xFF.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_data_setup_time(i2c_regs_t *I2Cx, uint32_t time)
{
    MODIFY_REG(I2Cx->SDA_SETUP, I2C_SDA_SETUP_SDA_SETUP, time);
}

/**
  * @brief  Get the SDA setup time when operating as a slave transmitter.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_SDA_SETUP         | SDA_SETUP                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value range between 0x02 and 0xFF.
  */
__STATIC_INLINE uint32_t ll_i2c_get_data_setup_time(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->SDA_SETUP, I2C_SDA_SETUP_SDA_SETUP));
}

/**
  * @brief  Set threshold of entries (or below) that trigger the TX_EMPTY interrupt
  * @note   TX FIFO threshold only can be configured after FIFO was enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_TX_TL             | TX_TL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_I2C_TX_FIFO_TH_EMPTY
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_7
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_tx_fifo_threshold(i2c_regs_t *I2Cx, uint32_t threshold)
{
    WRITE_REG(I2Cx->TX_TL, threshold);
}

/**
  * @brief  Get threshold of TX FIFO that triggers an THRE interrupt
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_TX_TL             | TX_TL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_TX_FIFO_TH_EMPTY
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_TX_FIFO_TH_CHAR_7
  */
__STATIC_INLINE uint32_t ll_i2c_get_tx_fifo_threshold(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TX_TL, I2C_TX_TL_TXTL));
}

/**
  * @brief  Set threshold of RX FIFO that triggers an RDA interrupt
  * @note   TX FIFO threshold only can be configured after FIFO was enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RX_TL             | RX_TL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_7
  *         @arg @ref LL_I2C_RX_FIFO_TH_FULL
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_rx_fifo_threshold(i2c_regs_t *I2Cx, uint32_t threshold)
{
    WRITE_REG(I2Cx->RX_TL, threshold);
}

/**
  * @brief  Get threshold of RX FIFO that triggers an RDA interrupt
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RX_TL             | RX_TL                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_1
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_2
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_3
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_4
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_5
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_6
  *         @arg @ref LL_I2C_RX_FIFO_TH_CHAR_7
  *         @arg @ref LL_I2C_RX_FIFO_TH_FULL
  */
__STATIC_INLINE uint32_t ll_i2c_get_rx_fifo_threshold(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->RX_TL, I2C_RX_TL_RXTL));
}

/**
  * @brief  Get FIFO Transmission Level
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_TXFLR             | TXFLR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Value range between 0x0 and 0x8.
  */
__STATIC_INLINE uint32_t ll_i2c_get_tx_fifo_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TXFLR, I2C_TXFLR_TXFLR));
}

/**
  * @brief  Get FIFO reception Level
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RXFLR             | RXFLR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Value range between 0x0 and 0x8.
  */
__STATIC_INLINE uint32_t ll_i2c_get_rx_fifo_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->RXFLR, I2C_RXFLR_RXFLR));
}

/**
  * @brief  Enable DMA reception requests.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ENABLE            | ABORT                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_transfer_abort(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->ENABLE, I2C_ENABLE_ABORT);
}

/**
  * @brief  Check if DMA reception requests are enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ENABLE            | ABORT                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_transfer_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->ENABLE, I2C_ENABLE_ABORT) == (I2C_ENABLE_ABORT));
}

/**
  * @brief  Get the transmit abort source.
  * @note   This can be used to retrieve source of TX_ABRT interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_TX_ABRT_SOURCE    | ABRT_USER_ABRT                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_TX_ABRT_SOURCE | ABRT_SLVRD_INTX
  *  IC_TX_ABRT_SOURCE | ABRT_SLV_ARBLOST
  *  IC_TX_ABRT_SOURCE | ABRT_SLVFLUSH_TXFIFO
  *  IC_TX_ABRT_SOURCE | ABRT_ARB_LOST
  *  IC_TX_ABRT_SOURCE | ABRT_MST_DIS
  *  IC_TX_ABRT_SOURCE | ABRT_10B_RD_NORSTRT
  *  IC_TX_ABRT_SOURCE | ABRT_SBYTE_NORSTRT
  *  IC_TX_ABRT_SOURCE | ABRT_HS_NORSTRT
  *  IC_TX_ABRT_SOURCE | ABRT_SBYTE_ACKDET
  *  IC_TX_ABRT_SOURCE | ABRT_HS_ACKDET
  *  IC_TX_ABRT_SOURCE | ABRT_GCALL_READ
  *  IC_TX_ABRT_SOURCE | ABRT_GCALL_NOACK
  *  IC_TX_ABRT_SOURCE | ABRT_TXDATA_NOACK
  *  IC_TX_ABRT_SOURCE | ABRT_10ADDR2_NOACK
  *  IC_TX_ABRT_SOURCE | ABRT_10ADDR1_NOACK
  *  IC_TX_ABRT_SOURCE | ABRT_7B_ADDR_NOACK
  *
  * @param  I2Cx I2C instance
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_I2C_ABRT_USER_ABRT
  *         @arg @ref LL_I2C_ABRT_SLVRD_INTX
  *         @arg @ref LL_I2C_ABRT_SLV_ARBLOST
  *         @arg @ref LL_I2C_ABRT_SLVFLUSH_TXFIFO
  *         @arg @ref LL_I2C_ABRT_ARB_LOST
  *         @arg @ref LL_I2C_ABRT_MST_DIS
  *         @arg @ref LL_I2C_ABRT_10B_RD_NORSTRT
  *         @arg @ref LL_I2C_ABRT_SBYTE_NORSTRT
  *         @arg @ref LL_I2C_ABRT_HS_NORSTRT
  *         @arg @ref LL_I2C_ABRT_SBYTE_ACKDET
  *         @arg @ref LL_I2C_ABRT_HS_ACKDET
  *         @arg @ref LL_I2C_ABRT_GCALL_READ
  *         @arg @ref LL_I2C_ABRT_GCALL_NOACK
  *         @arg @ref LL_I2C_ABRT_TXDATA_NOACK
  *         @arg @ref LL_I2C_ABRT_10ADDR2_NOACK
  *         @arg @ref LL_I2C_ABRT_10ADDR1_NOACK
  *         @arg @ref LL_I2C_ABRT_7B_ADDR_NOACK
  *
  * @note   @arg @ref LL_I2C_ABRT_TX_FLUSH_CNT can be used as a mask to get the
  *         number of Tx FIFO Data Commands which are flushed due to TX_ABRT
  *         interrupt.
  */
__STATIC_INLINE uint32_t ll_i2c_get_abort_source(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_REG(I2Cx->TX_ABRT_SOURCE) & (~I2C_TX_ABRT_SRC_TX_FLUSH_CNT));
}

/**
  * @brief  Get the number of Tx FIFO Data Commands which are flushed due to TX_ABRT interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_TX_ABRT_SOURCE    | TX_FLUSH_CNT                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Tx flush count.
  */
__STATIC_INLINE uint32_t ll_i2c_get_tx_flush_count(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TX_ABRT_SOURCE, I2C_TX_ABRT_SRC_TX_FLUSH_CNT) >> I2C_TX_ABRT_SRC_TX_FLUSH_CNT_Pos);
}

/** @} */

/** @defgroup I2C_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable specified interrupts.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | INTR_MASK_GEN_CALL                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  INTR_MASK | INTR_MASK_START_DET
  *  INTR_MASK | INTR_MASK_STOP_DET
  *  INTR_MASK | INTR_MASK_ACTIVITY
  *  INTR_MASK | INTR_MASK_RX_DONE
  *  INTR_MASK | INTR_MASK_TX_ABRT
  *  INTR_MASK | INTR_MASK_RD_REQ
  *  INTR_MASK | INTR_MASK_TX_EMPTY
  *  INTR_MASK | INTR_MASK_TX_OVER
  *  INTR_MASK | INTR_MASK_RX_FULL
  *  INTR_MASK | INTR_MASK_RX_OVER
  *  INTR_MASK | INTR_MASK_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_I2C_INTR_MASK_GEN_CALL
  *         @arg @ref LL_I2C_INTR_MASK_START_DET
  *         @arg @ref LL_I2C_INTR_MASK_STOP_DET
  *         @arg @ref LL_I2C_INTR_MASK_ACTIVITY
  *         @arg @ref LL_I2C_INTR_MASK_RX_DONE
  *         @arg @ref LL_I2C_INTR_MASK_TX_ABRT
  *         @arg @ref LL_I2C_INTR_MASK_RD_REQ
  *         @arg @ref LL_I2C_INTR_MASK_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_MASK_TX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_FULL
  *         @arg @ref LL_I2C_INTR_MASK_RX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_UNDER
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it(i2c_regs_t *I2Cx, uint32_t mask)
{
    SET_BITS(I2Cx->INTR_MASK, mask);
}

/**
  * @brief  Disable specified interrupts.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | INTR_MASK_GEN_CALL                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  INTR_MASK | INTR_MASK_START_DET
  *  INTR_MASK | INTR_MASK_STOP_DET
  *  INTR_MASK | INTR_MASK_ACTIVITY
  *  INTR_MASK | INTR_MASK_RX_DONE
  *  INTR_MASK | INTR_MASK_TX_ABRT
  *  INTR_MASK | INTR_MASK_RD_REQ
  *  INTR_MASK | INTR_MASK_TX_EMPTY
  *  INTR_MASK | INTR_MASK_TX_OVER
  *  INTR_MASK | INTR_MASK_RX_FULL
  *  INTR_MASK | INTR_MASK_RX_OVER
  *  INTR_MASK | INTR_MASK_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_I2C_INTR_MASK_GEN_CALL
  *         @arg @ref LL_I2C_INTR_MASK_START_DET
  *         @arg @ref LL_I2C_INTR_MASK_STOP_DET
  *         @arg @ref LL_I2C_INTR_MASK_ACTIVITY
  *         @arg @ref LL_I2C_INTR_MASK_RX_DONE
  *         @arg @ref LL_I2C_INTR_MASK_TX_ABRT
  *         @arg @ref LL_I2C_INTR_MASK_RD_REQ
  *         @arg @ref LL_I2C_INTR_MASK_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_MASK_TX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_FULL
  *         @arg @ref LL_I2C_INTR_MASK_RX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_UNDER
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it(i2c_regs_t *I2Cx, uint32_t mask)
{
    CLEAR_BITS(I2Cx->INTR_MASK, mask);
}

/**
  * @brief  Check if the specified interrupts are enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | INTR_MASK_GEN_CALL                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  INTR_MASK | INTR_MASK_START_DET
  *  INTR_MASK | INTR_MASK_STOP_DET
  *  INTR_MASK | INTR_MASK_ACTIVITY
  *  INTR_MASK | INTR_MASK_RX_DONE
  *  INTR_MASK | INTR_MASK_TX_ABRT
  *  INTR_MASK | INTR_MASK_RD_REQ
  *  INTR_MASK | INTR_MASK_TX_EMPTY
  *  INTR_MASK | INTR_MASK_TX_OVER
  *  INTR_MASK | INTR_MASK_RX_FULL
  *  INTR_MASK | INTR_MASK_RX_OVER
  *  INTR_MASK | INTR_MASK_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @param  mask This parameter can be a combination of the following values:
  *         @arg @ref LL_I2C_INTR_MASK_GEN_CALL
  *         @arg @ref LL_I2C_INTR_MASK_START_DET
  *         @arg @ref LL_I2C_INTR_MASK_STOP_DET
  *         @arg @ref LL_I2C_INTR_MASK_ACTIVITY
  *         @arg @ref LL_I2C_INTR_MASK_RX_DONE
  *         @arg @ref LL_I2C_INTR_MASK_TX_ABRT
  *         @arg @ref LL_I2C_INTR_MASK_RD_REQ
  *         @arg @ref LL_I2C_INTR_MASK_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_MASK_TX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_FULL
  *         @arg @ref LL_I2C_INTR_MASK_RX_OVER
  *         @arg @ref LL_I2C_INTR_MASK_RX_UNDER
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it(i2c_regs_t *I2Cx, uint32_t mask)
{
    return (READ_BITS(I2Cx->INTR_MASK, mask) == (mask));
}

/**
  * @brief  Enable MASTER_ON_HOLD interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | MST_ON_HOLD                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_master_on_hold(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_MST_ON_HOLD);
}

/**
  * @brief  Disable MASTER_ON_HOLD interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | MST_ON_HOLD                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_master_om_hold(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_MST_ON_HOLD);
}

/**
  * @brief  Check if the MASTER_ON_HOLD Interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | MST_ON_HOLD                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_master_on_hold(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_MST_ON_HOLD) == (I2C_INTR_MST_ON_HOLD));
}

/**
  * @brief  Enable RESTART_DET interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RESTART_DET                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_restart_det(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_RESTART_DET);
}

/**
  * @brief  Disable RESTART_DET interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RESTART_DET                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_restart_det(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_RESTART_DET);
}

/**
  * @brief  Check if the RESTART_DET Interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RESTART_DET                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_restart_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_RESTART_DET) == (I2C_INTR_RESTART_DET));
}

/**
  * @brief  Enable GEN_CALL interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | GEN_CALL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_gen_call(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_GEN_CALL);
}

/**
  * @brief  Disable GEN_CALL interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | GEN_CALL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_gen_call(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_GEN_CALL);
}

/**
  * @brief  Check if GEN_CALL interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | GEN_CALL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_gen_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_GEN_CALL) == (I2C_INTR_GEN_CALL));
}

/**
  * @brief  Enable START_DET received interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | START_DET                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_start_det(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_START_DET);
}

/**
  * @brief  Disable START_DET received interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | START_DET                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_start_det(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_START_DET);
}

/**
  * @brief  Check if START_DET received interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | START_DET                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_start_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_START_DET) == (I2C_INTR_START_DET));
}

/**
  * @brief  Enable STOP_DET interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | STOP_DET                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_stop_det(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_STOP_DET);
}

/**
  * @brief  Disable STOP_DET interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | STOP_DET                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_stop_det(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_STOP_DET);
}

/**
  * @brief  Check if STOP_DET interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | STOP_DET                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_stop_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_STOP_DET) == (I2C_INTR_STOP_DET));
}

/**
  * @brief  Enable ACTIVITY interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | ACTIVITY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_activity(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_ACTIVITY);
}

/**
  * @brief  Disable ACTIVITY interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | ACTIVITY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_activity(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_ACTIVITY);
}

/**
  * @brief  Check if ACTIVITY interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | ACTIVITY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_ACTIVITY) == (I2C_INTR_ACTIVITY));
}

/**
  * @brief  Enable RX_DONE interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_DONE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_done(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_DONE);
}

/**
  * @brief  Disable RX_DONE interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_DONE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_rx_done(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_DONE);
}

/**
  * @brief  Check if RX_DONE interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_DONE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enable_it_rx_done(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_DONE) == (I2C_INTR_RX_DONE));
}

/**
  * @brief  Enable TX_ABRT interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_ABRT                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_abort(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_ABRT);
}

/**
  * @brief  Disable TX_ABRT interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_ABRT                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_tx_abort(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_ABRT);
}

/**
  * @brief  Check if TX_ABRT interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_ABRT                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_tx_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_ABRT) == (I2C_INTR_TX_ABRT));
}

/**
  * @brief  Enable RD_REQ interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RD_REQ                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_read_req(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_RD_REQ);
}

/**
  * @brief  Disable RD_REQ interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RD_REQ                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_read_req(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_RD_REQ);
}

/**
  * @brief  Check if RD_REQ interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RD_REQ                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_read_req(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_RD_REQ) == (I2C_INTR_RD_REQ));
}

/**
  * @brief  Enable TX_EMPTY interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_EMPTY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_tx_empty(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_EMPTY);
}

/**
  * @brief  Disable TX_EMPTY interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_EMPTY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_tx_empty(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_EMPTY);
}

/**
  * @brief  Check if TX_EMPTY interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_EMPTY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_tx_empty(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_EMPTY) == (I2C_INTR_TX_EMPTY));
}

/**
  * @brief  Enable TX_OVER interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_tx_over(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_OVER);
}

/**
  * @brief  Disable TX_OVER interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_tx_over(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_OVER);
}

/**
  * @brief  Check if TX_OVER interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | TX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_tx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_TX_OVER) == (I2C_INTR_TX_OVER));
}

/**
  * @brief  Enable RX_FULL interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_FULL                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_full(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_FULL);
}

/**
  * @brief  Disable RX_FULL interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_FULL                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disbale_it_rx_full(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_FULL);
}

/**
  * @brief  Check if RX_FULL interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_FULL                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_ls_enabled_it_rx_full(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_FULL) == (I2C_INTR_RX_FULL));
}

/**
  * @brief  Enable RX_OVER interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_over(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_OVER);
}

/**
  * @brief  Disable RX_OVER interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_rx_over(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_OVER);
}

/**
  * @brief  Check if RX_OVER interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_rx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_OVER) == (I2C_INTR_RX_OVER));
}

/**
  * @brief  Enable RX_UNDER interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_UNDER                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_it_rx_under(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_UNDER);
}

/**
  * @brief  Disable RX_UNDER interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_UNDER                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_it_rx_under(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_UNDER);
}

/**
  * @brief  Check if RX_UNDER interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTR_MASK            | RX_UNDER                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_it_rx_under(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_MASK, I2C_INTR_RX_UNDER) == (I2C_INTR_RX_UNDER));
}

/** @} */

/** @defgroup I2C_LL_EF_FLAG_management FLAG_management
  * @{
  */

/**
  * @brief  Get I2C interrupt flags
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | MST_ON_HOLD                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_INTR_STAT | RESTART_DET
  *  IC_INTR_STAT | GEN_CALL
  *  IC_INTR_STAT | START_DET
  *  IC_INTR_STAT | STOP_DET
  *  IC_INTR_STAT | ACTIVITY
  *  IC_INTR_STAT | RX_DONE
  *  IC_INTR_STAT | TX_ABRT
  *  IC_INTR_STAT | RD_REQ
  *  IC_INTR_STAT | TX_EMPTY
  *  IC_INTR_STAT | TX_OVER
  *  IC_INTR_STAT | RX_FULL
  *  IC_INTR_STAT | RX_OVER
  *  IC_INTR_STAT | RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_I2C_INTR_STAT_MST_ON_HOLD
  *         @arg @ref LL_I2C_INTR_STAT_RESTART_DET
  *         @arg @ref LL_I2C_INTR_STAT_GEN_CALL
  *         @arg @ref LL_I2C_INTR_STAT_START_DET
  *         @arg @ref LL_I2C_INTR_STAT_STOP_DET
  *         @arg @ref LL_I2C_INTR_STAT_ACTIVITY
  *         @arg @ref LL_I2C_INTR_STAT_RX_DONE
  *         @arg @ref LL_I2C_INTR_STAT_TX_ABRT
  *         @arg @ref LL_I2C_INTR_STAT_RD_REQ
  *         @arg @ref LL_I2C_INTR_STAT_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_STAT_TX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_FULL
  *         @arg @ref LL_I2C_INTR_STAT_RX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_UNDER
  */
__STATIC_INLINE uint32_t ll_i2c_get_it_flag(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_REG(I2Cx->INTR_STAT));
}

/**
  * @brief  Get I2C RAW interrupt flags
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_MST_ON_HOLD                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_RAW_INTR_STAT | RAW_RESTART_DET
  *  IC_RAW_INTR_STAT | RAW_GEN_CALL
  *  IC_RAW_INTR_STAT | RAW_START_DET
  *  IC_RAW_INTR_STAT | RAW_STOP_DET
  *  IC_RAW_INTR_STAT | RAW_ACTIVITY
  *  IC_RAW_INTR_STAT | RAW_RX_DONE
  *  IC_RAW_INTR_STAT | RAW_TX_ABRT
  *  IC_RAW_INTR_STAT | RAW_RD_REQ
  *  IC_RAW_INTR_STAT | RAW_TX_EMPTY
  *  IC_RAW_INTR_STAT | RAW_TX_OVER
  *  IC_RAW_INTR_STAT | RAW_RX_FULL
  *  IC_RAW_INTR_STAT | RAW_RX_OVER
  *  IC_RAW_INTR_STAT | RAW_RX_UNDER
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value can be one or combination of the following values:
  *         @arg @ref LL_I2C_INTR_STAT_MST_ON_HOLD
  *         @arg @ref LL_I2C_INTR_STAT_RESTART_DET
  *         @arg @ref LL_I2C_INTR_STAT_GEN_CALL
  *         @arg @ref LL_I2C_INTR_STAT_START_DET
  *         @arg @ref LL_I2C_INTR_STAT_STOP_DET
  *         @arg @ref LL_I2C_INTR_STAT_ACTIVITY
  *         @arg @ref LL_I2C_INTR_STAT_RX_DONE
  *         @arg @ref LL_I2C_INTR_STAT_TX_ABRT
  *         @arg @ref LL_I2C_INTR_STAT_RD_REQ
  *         @arg @ref LL_I2C_INTR_STAT_TX_EMPTY
  *         @arg @ref LL_I2C_INTR_STAT_TX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_FULL
  *         @arg @ref LL_I2C_INTR_STAT_RX_OVER
  *         @arg @ref LL_I2C_INTR_STAT_RX_UNDER
  */
__STATIC_INLINE uint32_t ll_i2c_get_raw_it_flag(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_REG(I2Cx->RAW_INTR_STAT));
}

/**
  * @brief  Indicate the status of MST_ON_HOLD flag.
  * @note   RESET: Clear default value.
  *         SET  : When MST_ON_HOLD interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RAW_INTR_STAT        | MST_ON_HOLD                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_master_on_hold(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_MST_ON_HOLD) == (I2C_INTR_MST_ON_HOLD));
}

/**
  * @brief  Indicate the status of RAW_MST_ON_HOLD flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked MST_ON_HOLD interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_MST_ON_HOLD                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_master_on_hold(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_MST_ON_HOLD) == (I2C_INTR_MST_ON_HOLD));
}

/**
  * @brief  Indicate the status of RESTART_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RESTART_DET interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | RESTART_DET                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_restart_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_RESTART_DET) == (I2C_INTR_RESTART_DET));
}

/**
  * @brief  Indicate the status of RAW_RESTART_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RESTART_DET interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_RESTART_DET                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_restart_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_RESTART_DET) == (I2C_INTR_RESTART_DET));
}

/**
  * @brief  Indicate the status of GEN_CALL flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked GEN_CALL interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | GEN_CALL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_gen_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_GEN_CALL) == (I2C_INTR_GEN_CALL));
}

/**
  * @brief  Indicate the status of RAW_GEN_CALL flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked GEN_CALL interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_GEN_CALL                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_gen_call(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_GEN_CALL) == (I2C_INTR_GEN_CALL));
}

/**
  * @brief  Indicate the status of START_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked START_DET interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | START_DET                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_start_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_START_DET) == (I2C_INTR_START_DET));
}

/**
  * @brief  Indicate the status of RAW_START_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked START_DET interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_START_DET                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_start_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_START_DET) == (I2C_INTR_START_DET));
}

/**
  * @brief  Indicate the status of STOP_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked STOP_DET interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | STOP_DET                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_stop_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_STOP_DET) == (I2C_INTR_STOP_DET));
}

/**
  * @brief  Indicate the status of RAW_STOP_DET flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked STOP_DET interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_STOP_DET                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_stop_det(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_STOP_DET) == (I2C_INTR_STOP_DET));
}

/**
  * @brief  Indicate the status of ACTIVITY flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked ACTIVITY interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | ACTIVITY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_ACTIVITY) == (I2C_INTR_ACTIVITY));
}

/**
  * @brief  Indicate the status of RAW_ACTIVITY flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked ACTIVITY interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_ACTIVITY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_ACTIVITY) == (I2C_INTR_ACTIVITY));
}

/**
  * @brief  Indicate the status of RX_DONE flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_DONE interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | RX_DONE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_done(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_RX_DONE) == (I2C_INTR_RX_DONE));
}

/**
  * @brief  Indicate the status of RAW_RX_DONE flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_DONE interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_RX_DONE                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_done(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_RX_DONE) == (I2C_INTR_RX_DONE));
}

/**
  * @brief  Indicate the status of TX_ABRT flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked TX_ABRT interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | TX_ABRT                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_tx_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_TX_ABRT) == (I2C_INTR_TX_ABRT));
}

/**
  * @brief  Indicate the status of RAW_TX_ABRT flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked TX_ABRT interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_TX_ABRT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_tx_abort(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_TX_ABRT) == (I2C_INTR_TX_ABRT));
}

/**
  * @brief  Indicate the status of RD_REQ flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RD_REQ interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | RD_REQ                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_read_req(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_RD_REQ) == (I2C_INTR_RD_REQ));
}

/**
  * @brief  Indicate the status of RAW_RD_REQ flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RD_REQ interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_RD_REQ                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_read_req(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_RD_REQ) == (I2C_INTR_RD_REQ));
}

/**
  * @brief  Indicate the status of TX_EMPTY flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked TX_EMPTY interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | TX_EMPTY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_tx_empty(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_TX_EMPTY) == (I2C_INTR_TX_EMPTY));
}

/**
  * @brief  Indicate the status of RAW_TX_EMPTY flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked TX_EMPTY interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_TX_EMPTY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_tx_empty(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_TX_EMPTY) == (I2C_INTR_TX_EMPTY));
}

/**
  * @brief  Indicate the status of TX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked TX_OVER interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | TX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_tx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_TX_OVER) == (I2C_INTR_TX_OVER));
}

/**
  * @brief  Indicate the status of RAW_TX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked TX_OVER interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_TX_OVER                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_tx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_TX_OVER) == (I2C_INTR_TX_OVER));
}

/**
  * @brief  Indicate the status of RX_FULL flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_FULL interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | RX_FULL                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_full(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_RX_FULL) == (I2C_INTR_RX_FULL));
}

/**
  * @brief  Indicate the status of RAW_RX_FULL flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_FULL interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_RX_FULL                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_full(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_RX_FULL) == (I2C_INTR_RX_FULL));
}

/**
  * @brief  Indicate the status of RX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_OVER interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | RX_OVER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_RX_OVER) == (I2C_INTR_RX_OVER));
}

/**
  * @brief  Indicate the status of RAW_RX_OVER flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_OVER interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_RX_OVER                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_over(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_RX_OVER) == (I2C_INTR_RX_OVER));
}

/**
  * @brief  Indicate the status of RX_UNDER flag.
  * @note   RESET: Clear default value.
  *         SET  : When masked RX_UNDER interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_INTR_STAT         | RX_UNDER                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_rx_under(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->INTR_STAT, I2C_INTR_RX_UNDER) == (I2C_INTR_RX_UNDER));
}

/**
  * @brief  Indicate the status of RAW_RX_UNDER flag.
  * @note   RESET: Clear default value.
  *         SET  : When unmasked RX_UNDER interrupt is actived.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | RAW_RX_UNDER                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_raw_rx_under(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_RX_UNDER) == (I2C_INTR_RX_UNDER));
}

/**
  * @brief  Clear the combined interrupt, all individual interrupts, and the IC_TX_ABRT_SOURCE register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_INTR          | CLR_INTR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_intr(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_INTR);
    (void) tmpreg;
}

/**
  * @brief  Clear GEN_CALL flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_GEN_CALL      | CLR_GEN_CALL                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_gen_call(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_GEN_CALL);
    (void) tmpreg;
}

/**
  * @brief  Clear START_DET flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_START_DET     | CLR_START_DET                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_start_det(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_START_DET);
    (void) tmpreg;
}

/**
  * @brief  Clear STOP_DET flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_STOP_DET      | CLR_STOP_DET                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_stop_det(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_STOP_DET);
    (void) tmpreg;
}

/**
  * @brief  Clear ACTIVITY flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_ACTIVITY      | CLR_ACTIVITY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_activity(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_ACTIVITY);
    (void) tmpreg;
}

/**
  * @brief  Clear RX_DONE flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_RX_DONE       | CLR_RX_DONE                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_rx_done(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RX_DONE);
    (void) tmpreg;
}

/**
  * @brief  Clear TX_ABRT flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_TX_ABRT       | CLR_TX_ABRT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_tx_abort(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_TX_ABRT);
    (void) tmpreg;
}

/**
  * @brief  Clear RD_REQ flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_RD_REQ        | CLR_RD_REQ                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_read_req(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RD_REQ);
    (void) tmpreg;
}

/**
  * @brief  Clear TX_OVER flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_TX_OVER       | CLR_TX_OVER                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_tx_over(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_TX_OVER);
    (void) tmpreg;
}

/**
  * @brief  Clear RX_OVER flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_RX_OVER       | CLR_RX_OVER                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_rx_over(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RX_OVER);
    (void) tmpreg;
}

/**
  * @brief  Clear RX_UNDER flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CLR_RX_UNDER      | CLR_RX_UNDER                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_clear_flag_rx_under(i2c_regs_t *I2Cx)
{
    __IO uint32_t tmpreg;
    tmpreg = READ_REG(I2Cx->CLR_RX_UNDER);
    (void) tmpreg;
}

/**
  * @brief  Indicate the status of IC_STATUS Slave FSM Activity Status flag.
  * @note   RESET: Slave FSM is in IDLE state.
  *         SET  : When Slave FSM is not in IDLE state.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_STATUS            | SLV_ACTIVITY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_slave_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STATUS, I2C_STATUS_SLV_ACTIVITY) == (I2C_STATUS_SLV_ACTIVITY));
}

/**
  * @brief  Indicate the status of IC_STATUS Master FSM Activity Status flag.
  * @note   RESET: Master FSM is in IDLE state.
  *         SET  : When Master FSM is not in IDLE state.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_STATUS            | MST_ACTIVITY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_master_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STATUS, I2C_STATUS_MST_ACTIVITY) == (I2C_STATUS_MST_ACTIVITY));
}

/**
  * @brief  Indicate the status of IC_STATUS Receive FIFO Completely Full flag.
  * @note   RESET: Receive FIFO is not full.
  *         SET  : When Receive FIFO is full.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_STATUS            | RFF                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_rff(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STATUS, I2C_STATUS_RFF) == (I2C_STATUS_RFF));
}

/**
  * @brief  Indicate the status of IC_STATUS Receive FIFO Not Empty flag.
  * @note   RESET: Receive FIFO is empty.
  *         SET  : When Receive FIFO is not empty.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_STATUS            | RFNE                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_rfne(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STATUS, I2C_STATUS_RFNE) == (I2C_STATUS_RFNE));
}

/**
  * @brief  Indicate the status of IC_STATUS Transmit FIFO Completely Empty flag.
  * @note   RESET: Transmit FIFO is not empty.
  *         SET  : When Transmit FIFO is empty.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_STATUS            | TFE                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_tfe(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STATUS, I2C_STATUS_TFE) == (I2C_STATUS_TFE));
}

/**
  * @brief  Indicate the status of IC_STATUS Transmit FIFO Not Full flag.
  * @note   RESET: Transmit FIFO is full.
  *         SET  : When Transmit FIFO is not full.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_STATUS            | TFNF                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_tfnf(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STATUS, I2C_STATUS_TFNF) == (I2C_STATUS_TFNF));
}

/**
  * @brief  Indicate the status of IC_STATUS ACTIVITY flag.
  * @note   RESET:  I2C is idle.
  *         SET  :  When I2C is active.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_STATUS            | ACTIVITY                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_status_activity(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->STATUS, I2C_STATUS_ACTIVITY) == (I2C_STATUS_ACTIVITY));
}

/**
  * @brief  Indicate the status of Slave Received Data Lost flag.
  * @note   RESET:  Slave RX Data is not lost.
  *         SET  :  Slave RX Data is lost.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ENABLE_STATUS     | SLV_RX_LOST                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_slave_rx_data_lost(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->ENABLE_STATUS, I2C_ENABLE_STATUS_SLV_RX_LOST) == (I2C_ENABLE_STATUS_SLV_RX_LOST));
}

/**
  * @brief  Indicate the status of Slave Disabled While Busy flag.
  * @note   RESET:  Slave is disabled when it is idle.
  *         SET  :  Slave is disabled when it is active.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_ENABLE_STATUS     | SLV_DIS_WHL_BUSY                  |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_active_flag_slave_dis_whl_busy(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->ENABLE_STATUS, I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY) == (I2C_ENABLE_STATUS_SLV_DIS_WHL_BUSY));
}
/** @} */

/** @defgroup I2C_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA transmission requests.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_CR            | TDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Value range between 0 ~ 0x8.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_dma_req_tx(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->DMA_CR, I2C_DMA_CR_TDMAE);
}

/**
  * @brief  Disable DMA transmission requests.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_CR            | TDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_dma_req_tx(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->DMA_CR, I2C_DMA_CR_TDMAE);
}

/**
  * @brief  Check if DMA transmission requests are enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_CR            | TDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_dma_req_tx(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->DMA_CR, I2C_DMA_CR_TDMAE) == (I2C_DMA_CR_TDMAE));
}

/**
  * @brief  Enable DMA reception requests.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_CR            | RDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_enable_dma_req_rx(i2c_regs_t *I2Cx)
{
    SET_BITS(I2Cx->DMA_CR, I2C_DMA_CR_RDMAE);
}

/**
  * @brief  Disable DMA reception requests.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_CR            | RDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_disable_dma_req_rx(i2c_regs_t *I2Cx)
{
    CLEAR_BITS(I2Cx->DMA_CR, I2C_DMA_CR_RDMAE);
}

/**
  * @brief  Check if DMA reception requests are enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_CR            | RDMAE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_i2c_is_enabled_dma_req_rx(i2c_regs_t *I2Cx)
{
    return (READ_BITS(I2Cx->DMA_CR, I2C_DMA_CR_RDMAE) == (I2C_DMA_CR_RDMAE));
}

/**
  * @brief  Set level of TX FIFO that requests a DMA transmit.
  * @note   TX data level should equal to the watermark level, that is, the dma_tx_req
  *         signal is generated when the number of valid data entries in the transmit
  *         FIFO is equal to or below this field value, and TDMAE = 1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_TDLR          | DMATDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @param  level This parameter should range between 0x0 and 0x8.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_dma_tx_data_level(i2c_regs_t *I2Cx, uint32_t level)
{
    WRITE_REG(I2Cx->DMA_TDLR, level);
}

/**
  * @brief  Get level of TX FIFO that request a DMA transmit.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_TDLR          | DMATDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Returned value should range between 0x0 and 0x8.
  */
__STATIC_INLINE uint32_t ll_i2c_get_dma_tx_data_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->DMA_TDLR, I2C_DMA_TDLR_DMATDL));
}

/**
  * @brief  Set level of RX FIFO that requests a DMA receive.
  * @note   The watermark level = DMARDL + 1, that is, dma_rx_req is generated when
  *         the number of valid data entries in the receive FIFO is equal to or
  *         more than this field value + 1, and RDMAE = 1. For instance, when DMARDL
  *         is 0, then dma_rx_req is asserted when 1 or more data entries are present
  *         in the receive FIFO.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_RDLR          | DMARDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @param  level This parameter should range between 0x0 and 0x8.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_dma_rx_data_level(i2c_regs_t *I2Cx, uint32_t level)
{
    WRITE_REG(I2Cx->DMA_RDLR, level);
}

/**
  * @brief  Get level of RX FIFO that request a DMA receive.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DMA_RDLR          | DMARDL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Returned value should range between 0x0 and 0x8.
  */
__STATIC_INLINE uint32_t ll_i2c_get_dma_rx_data_level(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->DMA_RDLR, I2C_DMA_RDLR_DMARDL));
}

/**
  * @brief  Get the data register address used for DMA transfer
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DATA_CMD          | DAT                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t ll_i2c_dma_get_register_address(i2c_regs_t *I2Cx)
{
    return ((uint32_t) & (I2Cx->DATA_CMD));
}

/** @} */

/** @defgroup I2C_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Configure the slave address for transfer (master mode).
  * @note   The register IC_TAR can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_TAR               | TAR_ADDR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @param  slave_addr This parameter must be a value between 0x00 and 0x3F.
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_set_slave_address(i2c_regs_t *I2Cx, uint32_t slave_addr)
{
    MODIFY_REG(I2Cx->TAR, I2C_TAR_ADDR, slave_addr << I2C_TAR_ADDR_Pos);
}

/**
  * @brief  Get the slave address programmed for transfer (master mode).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_TAR               | TAR_ADDR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value between 0x0 and0x3F
  */
__STATIC_INLINE uint32_t ll_i2c_get_slave_address(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->TAR, I2C_TAR_ADDR) >> I2C_TAR_ADDR_Pos);
}

/**
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @note   The register IC_CON and IC_TAR can only be programmed when the I2C is disabled (ENABLE = 0).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_CON               | CON_10BITADDR_MST                 |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_TAR | TAR_ADDR
  *
  * @param  I2Cx I2C instance.
  * @param  slave_addr      Specifies the slave address to be programmed.
  * @param  slave_addr_size This parameter can be one of the following values:
  *         @arg @ref LL_I2C_ADDRESSING_MODE_7BIT
  *         @arg @ref LL_I2C_ADDRESSING_MODE_10BIT
  * @note   SlaveAddrSize in IC_CON register can only be programmed when the I2C is disabled (IC_ENABLE = 0).
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_handle_transfer(i2c_regs_t *I2Cx, uint32_t slave_addr, uint32_t slave_addr_size)
{
    MODIFY_REG(I2Cx->TAR, I2C_TAR_ADDR, slave_addr << I2C_TAR_ADDR_Pos);
    ll_i2c_set_master_addressing_mode(I2Cx, slave_addr_size);
}

/**
  * @brief  Indicate the value of transfer direction (slave mode).
  * @note   RESET: Write transfer, Slave enters in receiver mode.
  *         SET: Read transfer, Slave enters in transmitter mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_RAW_INTR_STAT     | INTR_RD_REQ                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_RAW_INTR_STAT | INTR_RX_FULL
  *
  * @param  I2Cx I2C instance.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2C_DIRECTION_WRITE
  *         @arg @ref LL_I2C_DIRECTION_READ
  */
__STATIC_INLINE uint32_t ll_i2c_get_transfer_direction(i2c_regs_t *I2Cx)
{
    return (uint32_t)(READ_BITS(I2Cx->RAW_INTR_STAT, I2C_INTR_RD_REQ | I2C_INTR_RX_FULL));
}

/**
  * @brief  Read Receive Data register.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DATA_CMD          | DAT                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  I2Cx I2C instance.
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t ll_i2c_receive_data8(i2c_regs_t *I2Cx)
{
    return (uint8_t)(READ_BITS(I2Cx->DATA_CMD, I2C_DATA_CMD_DAT));
}

/**
  * @brief  Write in Transmit Data Register .
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | IC_DATA_CMD          | STOP                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  IC_DATA_CMD | CMD
  *  IC_DATA_CMD | DAT
  *
  * @param  I2Cx I2C instance.
  * @param  data    Value range between 0x00 and 0xFF.
  * @param  cmd     This parameter can be one of the following values:
  *         @arg @ref LL_I2C_CMD_SLV_NONE
  *         @arg @ref LL_I2C_CMD_MST_WRITE
  *         @arg @ref LL_I2C_CMD_MST_READ
  *         @arg @ref LL_I2C_CMD_MST_GEN_STOP
  *         @arg @ref LL_I2C_CMD_MST_GEN_RESTART
  * @retval None.
  */
__STATIC_INLINE void ll_i2c_transmit_data8(i2c_regs_t *I2Cx, uint8_t data, uint32_t cmd)
{
    WRITE_REG(I2Cx->DATA_CMD, data | cmd);
}

/** @} */

/** @defgroup I2C_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize I2C registers (Registers restored to their default values).
  * @param  I2Cx I2C instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: I2C registers are de-initialized
  *          - ERROR: I2C registers are not de-initialized
  */
error_status_t ll_i2c_deinit(i2c_regs_t *I2Cx);

/**
  * @brief  Initialize I2C registers according to the specified
  *         parameters in p_i2c_init.
  * @param  I2Cx I2C instance
  * @param  p_i2c_init  Pointer to a ll_i2c_init_t structure that contains the configuration
  *                         information for the specified I2C peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: I2C registers are initialized according to p_i2c_init content
  *          - ERROR: Problem occurred during I2C Registers initialization
  */
error_status_t ll_i2c_init(i2c_regs_t *I2Cx, ll_i2c_init_t *p_i2c_init);

/**
  * @brief Set each field of a @ref ll_i2c_init_t type structure to default value.
  * @param p_i2c_init   Pointer to a @ref ll_i2c_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_i2c_struct_init(ll_i2c_init_t *p_i2c_init);

/** @} */

/** @} */

#endif /* I2C0 || I2C1 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_I2C_H__ */

/** @} */

/** @} */

/** @} */
