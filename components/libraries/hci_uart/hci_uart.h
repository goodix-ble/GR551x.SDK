/**
  ****************************************************************************************
  * @file    hci_uart.h
  * @author  BLE SDK Team
  * @brief   UART Driver for HCI over UART operation.
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

/** @addtogroup LIBRARIES Libraries
  * @{
  */

/**
  @addtogroup LIBRARIES_HCI_UART HCI HCI UART Driver.
  @{
  @brief Definitions and prototypes for HCI UART Driver.
 */

#ifndef HCI_UART_H_
#define HCI_UART_H_

#include <stdbool.h>
#include <stdint.h>

#include "gr55xx_hal.h"

/** @addtogroup HCI_UART_DRIVER_FUNCTIONS Functions
 * @{ */

/**
 ****************************************************************************************
 * @brief Initializes the UART0 to default values.
 ****************************************************************************************
 */
void hci_uart0_init(void);

/**
 ****************************************************************************************
 * @brief Initializes the UART1 to default values.
 ****************************************************************************************
 */
void hci_uart1_init(void);

#ifndef CFG_ROM
/**
 ****************************************************************************************
 * @brief Enable UART0 flow.
 *****************************************************************************************
 */
void hci_uart0_flow_on(void);

/**
 ****************************************************************************************
 * @brief Enable UART1 flow.
 *****************************************************************************************
 */
void hci_uart1_flow_on(void);

/**
 ****************************************************************************************
 * @brief Disable UART0 flow.
 *
 * @retval  true      if successful.
 * @retval  false     if failure.
 *****************************************************************************************
 */
bool hci_uart0_flow_off(void);

/**
 ****************************************************************************************
 * @brief Disable UART1 flow.
 *
 * @retval  true      if successful.
 * @retval  false     if failure.
 *****************************************************************************************
 */
bool hci_uart1_flow_off(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Finish current UART0 transfers
 *****************************************************************************************
 */
void hci_uart0_finish_transfers(void);

/**
 ****************************************************************************************
 * @brief Finish current UART1 transfers
 *****************************************************************************************
 */
void hci_uart1_finish_transfers(void);

/**
 ****************************************************************************************
 * @brief Starts a data reception of UART0.
 *
 * @param[out] bufptr   Pointer to the RX buffer
 * @param[in]  size     Size of the expected reception
 * @param[in]  callback Pointer to the function called back when transfer finished
 * @param[in]  dummy    Dummy data pointer returned to callback when reception is finished
 *****************************************************************************************
 */
void hci_uart0_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

/**
 ****************************************************************************************
 * @brief Starts a data reception of UART1.
 *
 * @param[out] bufptr   Pointer to the RX buffer
 * @param[in]  size     Size of the expected reception
 * @param[in]  callback Pointer to the function called back when transfer finished
 * @param[in]  dummy    Dummy data pointer returned to callback when reception is finished
 *****************************************************************************************
 */
void hci_uart1_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

/**
 ****************************************************************************************
 * @brief Starts a data transmission of UART0.
 *
 * @param[in] bufptr   Pointer to the TX buffer
 * @param[in] size     Size of the transmission
 * @param[in] callback Pointer to the function called back when transfer finished
 * @param[in] dummy    Dummy data pointer returned to callback when transmission is finished
 *****************************************************************************************
 */
void hci_uart0_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

/**
 ****************************************************************************************
 * @brief Starts a data transmission of UART1.
 *
 * @param[in] bufptr   Pointer to the TX buffer
 * @param[in] size     Size of the transmission
 * @param[in] callback Pointer to the function called back when transfer finished
 * @param[in] dummy    Dummy data pointer returned to callback when transmission is finished
 *****************************************************************************************
 */
void hci_uart1_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

#if defined(CFG_ROM)
/**
 ****************************************************************************************
 * @brief Poll UART0 on reception and transmission.
 *
 * This function is used to poll UART0 for reception and transmission.
 * It is used when IRQ are not used to detect incoming bytes.
 *****************************************************************************************
 */
void hci_uart0_poll(void);

/**
 ****************************************************************************************
 * @brief Poll UART1 on reception and transmission.
 *
 * This function is used to poll UART1 for reception and transmission.
 * It is used when IRQ are not used to detect incoming bytes.
 *****************************************************************************************
 */
void hci_uart1_poll(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Handle UART0 interrupt request.
 *
 * This function is used to handle UART0 interrupt request.
 * It must to be called in UART0_IRQHandler().
 *****************************************************************************************
 */
void hci_uart0_irq_handler(void);

/**
 ****************************************************************************************
 * @brief Handle UART1 interrupt request.
 *
 * This function is used to handle UART1 interrupt request.
 * It must to be called in UART1_IRQHandler().
 *****************************************************************************************
 */
void hci_uart1_irq_handler(void);

/**
 ****************************************************************************************
 * @brief HCI UART Tx Transfer completed callback.
 *
 * It must to be called in hal_uart_tx_cplt_callback().
 *****************************************************************************************
 */
void hci_uart_tx_cplt_callback(uart_handle_t *huart);

/**
 ****************************************************************************************
 * @brief HCI UART Rx Transfer completed callback.
 *
 * It must to be called in hal_uart_rx_cplt_callback().
 *****************************************************************************************
 */
void hci_uart_rx_cplt_callback(uart_handle_t *huart);

/**
 ****************************************************************************************
 * @brief HCI UART Error callback.
 *
 * It must to be called in hal_uart_error_callback().
 *****************************************************************************************
 */
void hci_uart_error_callback(uart_handle_t *huart);

/**
 ****************************************************************************************
 * @brief BLE HCI uart init.
 *
 * If users want to support dtm test, this function should be called.
 *****************************************************************************************
 */
void ble_hci_uart_init(void);

/**
 ****************************************************************************************
 * @brief BLE stack init handler function.
 *
 * BLE Stack init complete, this function should be called.
 *****************************************************************************************
 */
void ble_hci_stack_init_handle(void);

/** @} */

#endif /* HCI_UART_H_ */

/** @} */
/** @} */
