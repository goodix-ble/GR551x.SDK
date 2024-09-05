/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
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
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "app_log.h"
#include "app_io.h"
#include "app_gpiote.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_GPIO_KEY0_PIN        APP_KEY_UP_PIN
#define APP_GPIO_KEY0_TYPE       APP_KEY_UP_IO_TYPE

#define APP_GPIO_KEY1_PIN        APP_KEY_OK_PIN
#define APP_GPIO_KEY1_TYPE       APP_KEY_OK_IO_TYPE

/*
 * GLOBAL FUNCTION DECLARATION
 ****************************************************************************************
 */
void app_io_event_handler(app_io_evt_t *p_evt);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

const app_gpiote_param_t gpiote_param[] =
{
    {APP_GPIO_KEY0_TYPE, APP_GPIO_KEY0_PIN, APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, app_io_event_handler},
    {APP_GPIO_KEY1_TYPE, APP_GPIO_KEY1_PIN, APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, app_io_event_handler},
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_io_event_handler(app_io_evt_t *p_evt)
{
    app_io_pin_state_t pin_level = APP_IO_PIN_RESET;

    if (p_evt->arg != NULL)
    {
        printf("%s\r\n", (char *)p_evt->arg);
    }

    if (p_evt->pin == APP_GPIO_KEY0_PIN)
    {
        pin_level = app_io_read_pin(APP_GPIO_KEY0_TYPE, APP_GPIO_KEY0_PIN);
        if (pin_level == APP_IO_PIN_RESET)
        {
            delay_ms(20);
            do
            {
                pin_level = app_io_read_pin(APP_GPIO_KEY0_TYPE, APP_GPIO_KEY0_PIN);
            } while(pin_level == APP_IO_PIN_SET);

            printf("\r\nKEY0 pressed.\r\n");
        }
    }
    if (p_evt->pin == APP_GPIO_KEY1_PIN)
    {
        pin_level = app_io_read_pin(APP_GPIO_KEY1_TYPE, APP_GPIO_KEY1_PIN);
        if (pin_level == APP_IO_PIN_RESET)
        {
            delay_ms(20);
            do
            {
                pin_level = app_io_read_pin(APP_GPIO_KEY1_TYPE, APP_GPIO_KEY1_PIN);
            } while(pin_level == APP_IO_PIN_SET);

            printf("\r\nKEY1 pressed.\r\n");
        }
    }
}

void gpio_output_input_demo(void)
{
    uint16_t ret;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_io_pin_state_t pin_val;
    int cnt = 10;

    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = APP_GPIO_PIN0;
    io_init.mux  = APP_IO_MUX;
    ret = app_io_init(APP_GPIO_PIN0_TYPE, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("APP_GPIO_PIN0 init failed.\r\n");
    }

    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = APP_GPIO_PIN1;
    io_init.mux  = APP_IO_MUX;
    ret = app_io_init(APP_GPIO_PIN1_TYPE, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("APP_GPIO_PIN1 init failed.\r\n");
    }

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_INPUT;
    io_init.pin  = APP_GPIO_PIN2;
    io_init.mux  = APP_IO_MUX;
    ret = app_io_init(APP_GPIO_PIN2_TYPE, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("APP_GPIO_PIN2 init failed.\r\n");
    }

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_INPUT;
    io_init.pin  = APP_GPIO_PIN3;
    io_init.mux  = APP_IO_MUX;
    ret = app_io_init(APP_GPIO_PIN3_TYPE, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("APP_GPIO_PIN3 init failed.\r\n");
    }

    while (cnt--)
    {
        app_io_write_pin(APP_GPIO_PIN0_TYPE, APP_GPIO_PIN0, APP_IO_PIN_SET);
        pin_val = app_io_read_pin(APP_GPIO_PIN2_TYPE, APP_GPIO_PIN2);
        printf("APP_GPIO_PIN0 set high, APP_GPIO_PIN2 input value=%d\r\n", pin_val);

        app_io_write_pin(APP_GPIO_PIN0_TYPE, APP_GPIO_PIN0, APP_IO_PIN_RESET);
        pin_val = app_io_read_pin(APP_GPIO_PIN2_TYPE, APP_GPIO_PIN2);
        printf("APP_GPIO_PIN0 set low, APP_GPIO_PIN2 input value=%d\r\n", pin_val);

        app_io_toggle_pin(APP_GPIO_PIN1_TYPE, APP_GPIO_PIN1);
        pin_val = app_io_read_pin(APP_GPIO_PIN3_TYPE, APP_GPIO_PIN3);
        printf("APP_GPIO_PIN1 toggle, APP_GPIO_PIN3 input value=%d\r\n", pin_val);
    }
}

void gpio_interrupt_demo(void)
{
    uint16_t ret;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_IT_FALLING;
    io_init.pin  = APP_GPIO_KEY0_PIN;
    io_init.mux  = APP_IO_MUX;

    ret = app_io_event_register_cb(APP_GPIO_KEY0_TYPE, &io_init, app_io_event_handler, "KEY0 pin interrupt");
    if (ret != APP_DRV_SUCCESS)
    {
        printf("APP_GPIO_KEY0 init failed.\r\n");
    }

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_IT_FALLING;
    io_init.pin  = APP_GPIO_KEY1_PIN;
    io_init.mux  = APP_IO_MUX;
    ret = app_io_event_register_cb(APP_GPIO_KEY1_TYPE, &io_init, app_io_event_handler, "KEY1 pin interrupt");
    if (ret != APP_DRV_SUCCESS)
    {
        printf("APP_GPIO_KEY0 init failed.\r\n");
    }
}

void gpiote_demo(void)
{
    app_gpiote_init(gpiote_param, sizeof(gpiote_param) / sizeof(app_gpiote_param_t));
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("************************************************************\r\n");
    printf("*                   GPIO_APP example.                      *\r\n");
    printf("*                                                          *\r\n");
    printf("*           APP_GPIO_KEY0  ----->   KEY0                   *\r\n");
    printf("*           APP_GPIO_KEY1  ----->   KEY1                   *\r\n");
    printf("*           APP_GPIO_PIN0 <----->   APP_GPIO_PIN2          *\r\n");
    printf("*           APP_GPIO_PIN1 <----->   APP_GPIO_PIN3          *\r\n");
    printf("*                                                          *\r\n");
    printf("* Please connect APP_GPIO_KEY0/APP_GPIO_KEY1 to KEY1/KEY2. *\r\n");
    printf("* Please connect APP_GPIO_PIN0 to APP_GPIO_PIN2.           *\r\n");
    printf("* Please connect APP_GPIO_PIN1 to APP_GPIO_PIN3.           *\r\n");
    printf("* This sample will show the GPIO interrupts from buttons.  *\r\n");
    printf("* Please press KEY1 or KEY2.                               *\r\n");
    printf("************************************************************\r\n");

    gpio_output_input_demo();
    gpio_interrupt_demo();
    gpiote_demo();
    
    while(1);
}
