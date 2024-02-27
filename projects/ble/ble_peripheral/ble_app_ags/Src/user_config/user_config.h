/**
 ****************************************************************************************
 *
 * @file user_config.h
 *
 * @brief Custom configuration file for AGS applications.
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
 *****************************************************************************************
 */

/*
 * DEFINES
 *****************************************************************************************
 */
#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

/**@brief The name that the user uses to identify the gadget. 
 *        This name must match the friendly name of the Bluetooth device. 
 */
#define DEVICE_NAME "Goodix_AGS"

/**@brief A general name for the type of Alexa gadget.
 */
#define USER_GADGET_MODEL_NAME "Goodix Gadget"

/**@brief The name of the device manufacturer.
 *        This value cannot exceed 128 characters.
 */
#define USER_GADGET_MANU_NAME  "Goodix"

/**@brief A human-readable description of the gadget. 
 *        This value cannot exceed 128 characters.
 */
#define USER_GADGET_DEVICE_DESCRIPTION "GR5xxx SK"

/**@brief The version of the firmware that is running on the gadget.
 *        Which is used to determine whether the gadget needs an OTA update.
 */
#define USER_GADGET_FW_VERSION "1"

/**@brief The device secret algorithm. 
 *        The only valid value is currently 1, which means that the algorithm is SHA256.
 */
#define USER_GADGET_DEV_TOKEN_ENC_TYPE "1"

/**@brief The MAC address of the gadget.
 *        This value should be in hexadecimal format.
 */
#define USER_GADGET_RADIO_ADDR "default"

/**@brief The type of the gadget. 
 *        This is the Amazon ID that is displayed on the gadget's product page int the
 *        https://developer.amazon.com/alexa/console/avs/home.
 */
#define USER_GADGET_AMAZON_ID "A1NQZWYY6JCV4U"

/**@brief Alexa Gadget device secret. 
 *        Device secret allocated by Amazon. As this is considered a secret value,
 *        enable flash readback protection accordingly.
 */
#define USER_GADGET_DEVICE_SECRET "82FD3552615BE5C92D62E041B94934D52259C1E1871E864F97250A9A6BF5F02E"

/**@brief Gadgets maxinmum transaction size.
 *        Maximum size of a single transaction. This should be big enough to cover the Discovery handshake,
 *        which is normally the largest transaction.
*/
#define USER_GADGET_TRANSACTION_BUF_SIZE 600

/**@brief Enable Alexa Gadgets OTA protocol. */
#define USER_GADGET_OTA_ENABLE 0

/**
 * @defgroup AGS_USER_CONFIG_MACRO Defines
 * @{
 */
#define USER_GADGET_CAPABILITY_ALERTS_ENABLE        1    /**< Support the Alerts feature. */
#define USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE 1    /**< Support the Alerts feature. */
#define USER_GADGET_CAPABILITY_STATELISTENER_ENABLE 1    /**< Support the Alerts feature. */
#define USER_GADGET_CAPABILITY_MUSICDATA_ENABLE     1    /**< Support the Alerts feature. */
#define USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE    1    /**< Support the Alerts feature. */
#define USER_GADGET_CAPABILITY_CUSTOM_ENABLE        0    /**< Support the Alerts feature. */
/** @} */

/**@brief Set Custom interface namespace.
 * Specify name of the custom interface.
 */
#define USER_GADGET_CUSTOM_NAMESPACE "Custom.TestGadget"

#endif //__USER_CONFIG_H__
