#include "io_pin_monitor.h"
#include "app_pwr_mgmt.h"
#include "app_log.h"
#if defined(SOC_GR5405) || defined(SOC_SGP)
#include "hal_gpio.h"
#if !defined(SOC_SGP)
#include "hal_msio.h"
#include "hal_aon_gpio.h"
#endif
#elif defined(SOC_GR533X)
#include "gr533x_hal_gpio.h"
#include "gr533x_hal_msio.h"
#include "gr533x_hal_aon_gpio.h"
#else
#include "gr55xx_hal_gpio.h"
#include "gr55xx_hal_msio.h"
#include "gr55xx_hal_aon_gpio.h"
#endif

typedef struct {
    gpio_regs_t *port;
    uint32_t pin_num;
    uint32_t *direction_state;
    uint32_t *pull_state;
    uint32_t *input_state;
    uint32_t *output_state;
} gpio_monitor_t;

#if defined(GPIO0_MONITOR_ENABLE)
static volatile uint32_t s_gpio0_direction_state;
static volatile uint32_t s_gpio0_pull_state;
static volatile uint32_t s_gpio0_input_state;
static volatile uint32_t s_gpio0_output_state;
#endif

#if defined(GPIO1_MONITOR_ENABLE)
static volatile uint32_t s_gpio1_direction_state;
static volatile uint32_t s_gpio1_pull_state;
static volatile uint32_t s_gpio1_input_state;
static volatile uint32_t s_gpio1_output_state;
#endif

#if defined(GPIO2_MONITOR_ENABLE)
static volatile uint32_t s_gpio2_direction_state;
static volatile uint32_t s_gpio2_pull_state;
static volatile uint32_t s_gpio2_input_state;
static volatile uint32_t s_gpio2_output_state;
#endif

#if defined(MSIO_MONITOR_ENABLE)
static volatile uint32_t s_msio_direction_state;
static volatile uint16_t s_msio_mode_state;
static volatile uint32_t s_msio_pull_state;
static volatile uint16_t s_msio_input_state;
static volatile uint16_t s_msio_output_state;
#endif

#if defined(AON_GPIO_MONITOR_ENABLE)
static volatile uint16_t s_aon_gpio_direction_state;
static volatile uint16_t s_aon_gpio_pull_state;
static volatile uint16_t s_aon_gpio_input_state;
static volatile uint16_t s_aon_gpio_output_state;
#endif

static void io_pin_monitor_schedule(void);
static pwr_mgmt_dev_state_t pwr_enter_sleep_check_io_pin_monitor(void);

#if defined(SOC_GR533X) || defined(SOC_GR5X25) || defined(SOC_GR5526)
extern pwr_mgmt_dev_state_t pwr_enter_sleep_check_new(void);
#else
extern pwr_mgmt_dev_state_t pwr_enter_sleep_check(void);
#endif

void io_pin_monitor_init(void)
{
    pwr_mgmt_set_callback(pwr_enter_sleep_check_io_pin_monitor, NULL);
}

void io_pin_monitor_output_raw(void)
{
#if defined(GPIO0_MONITOR_ENABLE)
    APP_LOG_INFO("GPIO0:===============");
    APP_LOG_INFO("GPIO0 PIN NUM = %d", GPIO0_PIN_NUM);
    APP_LOG_INFO("direction_state= 0x%08X", s_gpio0_direction_state);
    APP_LOG_INFO("pull_state     = 0x%08X", s_gpio0_pull_state);
    APP_LOG_INFO("input_state    = 0x%04X", s_gpio0_input_state);
    APP_LOG_INFO("output_state   = 0x%04X", s_gpio0_output_state);
    APP_LOG_INFO("GPIO0:===============");
    s_gpio0_direction_state = 0;
    s_gpio0_pull_state = 0;
    s_gpio0_input_state = 0;
    s_gpio0_output_state = 0;
#endif

#if defined(GPIO1_MONITOR_ENABLE)
    APP_LOG_INFO("GPIO1:===============");
    APP_LOG_INFO("GPIO1 PIN NUM = %d", GPIO1_PIN_NUM);
    APP_LOG_INFO("direction_state= 0x%08X", s_gpio1_direction_state);
    APP_LOG_INFO("pull_state     = 0x%08X", s_gpio1_pull_state);
    APP_LOG_INFO("input_state    = 0x%04X", s_gpio1_input_state);
    APP_LOG_INFO("output_state   = 0x%04X", s_gpio1_output_state);
    APP_LOG_INFO("GPIO1:===============");
    s_gpio1_direction_state = 0;
    s_gpio1_pull_state = 0;
    s_gpio1_input_state = 0;
    s_gpio1_output_state = 0;
#endif

#if defined(GPIO2_MONITOR_ENABLE)
    APP_LOG_INFO("GPIO2:===============");
    APP_LOG_INFO("GPIO2 PIN NUM = %d", GPIO2_PIN_NUM);
    APP_LOG_INFO("direction_state= 0x%08X", s_gpio2_direction_state);
    APP_LOG_INFO("pull_state     = 0x%08X", s_gpio2_pull_state);
    APP_LOG_INFO("input_state    = 0x%04X", s_gpio2_input_state);
    APP_LOG_INFO("output_state   = 0x%04X", s_gpio2_output_state);
    APP_LOG_INFO("GPIO2:===============");
    s_gpio2_direction_state = 0;
    s_gpio2_pull_state = 0;
    s_gpio2_input_state = 0;
    s_gpio2_output_state = 0;
#endif

#if defined(MSIO_MONITOR_ENABLE)
    APP_LOG_INFO("MSIO:===============");
    APP_LOG_INFO("MSIO PIN NUM = %d", MSIO_PIN_NUM);
    APP_LOG_INFO("direction_state= 0x%08X", s_msio_direction_state);
    APP_LOG_INFO("pull_state     = 0x%08X", s_msio_pull_state);
    APP_LOG_INFO("input_state    = 0x%04X", s_msio_input_state);
    APP_LOG_INFO("output_state   = 0x%04X", s_msio_output_state);
    APP_LOG_INFO("mode_state     = 0x%04X", s_msio_mode_state);
    APP_LOG_INFO("MSIO:===============");
    s_msio_direction_state = 0;
    s_msio_mode_state = 0;
    s_msio_pull_state = 0;
    s_msio_input_state = 0;
    s_msio_output_state = 0;
#endif

#if defined(AON_GPIO_MONITOR_ENABLE)
    APP_LOG_INFO("AON_GPIO:===============");
    APP_LOG_INFO("AON_GPIO PIN NUM = %d", AON_GPIO_PIN_NUM);
    APP_LOG_INFO("direction_state= 0x%04X", s_aon_gpio_direction_state);
    APP_LOG_INFO("pull_state     = 0x%04X", s_aon_gpio_pull_state);
    APP_LOG_INFO("input_state    = 0x%04X", s_aon_gpio_input_state);
    APP_LOG_INFO("output_state   = 0x%04X", s_aon_gpio_output_state);
    APP_LOG_INFO("AON_GPIO:===============");
    s_aon_gpio_direction_state = 0;
    s_aon_gpio_pull_state = 0;
    s_aon_gpio_input_state = 0;
    s_aon_gpio_output_state = 0;
#endif

    app_log_flush();
}

void io_pin_monitor_output(void)
{
    const char* direction_str[] = {
        "none",
        "input",
        "output",
        "mux"
    };

    const char* pull_str[] = {
        "nopull",
        "pullup",
        "pulldown",
        "error"
    };

    const char* input_str[] = {
        "low",
        "high",
    };

    const char* output_str[] = {
        "low",
        "high",
    };

    const char* mode_str[] = {
        "digital",
        "analog",
    };

#if defined(GPIO0_MONITOR_ENABLE)
    APP_LOG_INFO("GPIO0:===============");
    APP_LOG_INFO("GPIO0 PIN NUM = %d", GPIO0_PIN_NUM);
    APP_LOG_INFO("             direction   pull       input   output");
    for (uint8_t index = 0; index < GPIO0_PIN_NUM; index++)
    {
        uint8_t direction = (s_gpio0_direction_state >> (index * 2)) & 0x03;
        uint8_t pull = (s_gpio0_pull_state >> (index * 2)) & 0x03;
        uint8_t input = (s_gpio0_input_state >> index) & 0x1;
        uint8_t output = (s_gpio0_output_state >> index) & 0x1;
        APP_LOG_INFO("GPIO0 PIN%2d: %-6s      %-8s   %-4s    %-4s", index, direction_str[direction], pull_str[pull], input_str[input], output_str[output]);
    }
    APP_LOG_INFO("GPIO0:===============\r\n");
    app_log_flush();
    s_gpio0_direction_state = 0;
    s_gpio0_pull_state = 0;
    s_gpio0_input_state = 0;
    s_gpio0_output_state = 0;
#endif

#if defined(GPIO1_MONITOR_ENABLE)
    APP_LOG_INFO("GPIO1:===============");
    APP_LOG_INFO("GPIO1 PIN NUM = %d", GPIO1_PIN_NUM);
    APP_LOG_INFO("             direction   pull       input   output");
    for (uint8_t index = 0; index < GPIO1_PIN_NUM; index++)
    {
        uint8_t direction = (s_gpio1_direction_state >> (index * 2)) & 0x03;
        uint8_t pull = (s_gpio1_pull_state >> (index * 2)) & 0x03;
        uint8_t input = (s_gpio1_input_state >> index) & 0x1;
        uint8_t output = (s_gpio1_output_state >> index) & 0x1;
        APP_LOG_INFO("GPIO1 PIN%2d: %-6s      %-8s   %-4s    %-4s", index, direction_str[direction], pull_str[pull], input_str[input], output_str[output]);
    }
    APP_LOG_INFO("GPIO1:===============\r\n");
    app_log_flush();
    s_gpio1_direction_state = 0;
    s_gpio1_pull_state = 0;
    s_gpio1_input_state = 0;
    s_gpio1_output_state = 0;
#endif

#if defined(GPIO2_MONITOR_ENABLE)
    APP_LOG_INFO("GPIO2:===============");
    APP_LOG_INFO("GPIO2 PIN NUM = %d", GPIO2_PIN_NUM);
    APP_LOG_INFO("             direction   pull       input   output");
    for (uint8_t index = 0; index < GPIO2_PIN_NUM; index++)
    {
        uint8_t direction = (s_gpio2_direction_state >> (index * 2)) & 0x03;
        uint8_t pull = (s_gpio2_pull_state >> (index * 2)) & 0x03;
        uint8_t input = (s_gpio2_input_state >> index) & 0x1;
        uint8_t output = (s_gpio2_output_state >> index) & 0x1;
        APP_LOG_INFO("GPIO2 PIN%2d: %-6s      %-8s   %-4s    %-4s", index, direction_str[direction], pull_str[pull], input_str[input], output_str[output]);
    }
    APP_LOG_INFO("GPIO2:===============\r\n");
    app_log_flush();
    s_gpio2_direction_state = 0;
    s_gpio2_pull_state = 0;
    s_gpio2_input_state = 0;
    s_gpio2_output_state = 0;
#endif

#if defined(MSIO_MONITOR_ENABLE)
    APP_LOG_INFO("MSIO:===============");
    APP_LOG_INFO("MSIO PIN NUM = %d", MSIO_PIN_NUM);
    APP_LOG_INFO("            direction   pull       input   output   mode");
    for (uint8_t index = 0; index < MSIO_PIN_NUM; index++)
    {
        uint8_t direction = (s_msio_direction_state >> (index * 2)) & 0x03;
        uint8_t pull = (s_msio_pull_state >> (index * 2)) & 0x03;
        uint8_t input = (s_msio_input_state >> index) & 0x1;
        uint8_t output = (s_msio_output_state >> index) & 0x1;
        uint8_t mode = (s_msio_mode_state >> index) & 0x1;
        APP_LOG_INFO("MSIO PIN%2d: %-6s      %-8s   %-4s    %-4s     %-7s", index, direction_str[direction], pull_str[pull], input_str[input], output_str[output], mode_str[mode]);
    }
    APP_LOG_INFO("MSIO:===============\r\n");
    app_log_flush();
    s_msio_direction_state = 0;
    s_msio_mode_state = 0;
    s_msio_pull_state = 0;
    s_msio_input_state = 0;
    s_msio_output_state = 0;
#endif

#if defined(AON_GPIO_MONITOR_ENABLE)
    APP_LOG_INFO("AON_GPIO:===============");
    APP_LOG_INFO("AON_GPIO PIN NUM = %d", AON_GPIO_PIN_NUM);
    APP_LOG_INFO("                direction   pull       input   output");
    for (uint8_t index = 0; index < AON_GPIO_PIN_NUM; index++)
    {
        uint8_t direction = (s_aon_gpio_direction_state >> (index * 2)) & 0x03;
        uint8_t pull = (s_aon_gpio_pull_state >> (index * 2)) & 0x03;
        uint8_t input = (s_aon_gpio_input_state >> index) & 0x1;
        uint8_t output = (s_aon_gpio_output_state >> index) & 0x1;
        APP_LOG_INFO("AON_GPIO PIN%2d: %-6s      %-8s   %-4s    %-4s", index, direction_str[direction], pull_str[pull], input_str[input], output_str[output]);
    }
    APP_LOG_INFO("AON_GPIO:===============\r\n");
    app_log_flush();
    s_aon_gpio_direction_state = 0;
    s_aon_gpio_pull_state = 0;
    s_aon_gpio_input_state = 0;
    s_aon_gpio_output_state = 0;
#endif
}

static void io_pin_monitor_gpio(gpio_monitor_t *gpio_monitor)
{
    for (uint8_t index = 0; index < gpio_monitor->pin_num; index++)
    {
#if defined(SOC_GR5515) || defined(SOC_GR5526)
        *(gpio_monitor->direction_state) |= ((ll_gpio_get_pin_mode(gpio_monitor->port, LL_GPIO_PIN_0 << index) + 1) << (index * 2));
        uint32_t pull_state = ll_gpio_get_pin_pull(gpio_monitor->port, LL_GPIO_PIN_0 << index);
        switch (pull_state)
        {
            case LL_GPIO_PULL_NO:
                pull_state = 0;
                break;
            case LL_GPIO_PULL_UP:
                pull_state = 1;
                break;
            case LL_GPIO_PULL_DOWN:
                pull_state = 2;
                break;
            default:
                pull_state = 3;
                break;
        }
        *(gpio_monitor->pull_state) |= (pull_state << (index * 2));
#else
        *(gpio_monitor->direction_state) |= ((ll_gpio_get_pin_mode(gpio_monitor->port, LL_GPIO_PIN_0 << index)) << (index * 2));
        *(gpio_monitor->pull_state) |= ((ll_gpio_get_pin_pull(gpio_monitor->port, LL_GPIO_PIN_0 << index)) << (index * 2));
#endif
    }
    *(gpio_monitor->input_state) = ll_gpio_read_input_port(gpio_monitor->port);
    *(gpio_monitor->output_state) = ll_gpio_read_output_port(gpio_monitor->port);
}

static void io_pin_monitor_schedule(void)
{
#if defined(GPIO0_MONITOR_ENABLE)
    gpio_monitor_t gpio_monitor;
    gpio_monitor.port = GPIO0;
    gpio_monitor.direction_state = (uint32_t *)&s_gpio0_direction_state;
    gpio_monitor.pull_state = (uint32_t *)&s_gpio0_pull_state;
    gpio_monitor.input_state = (uint32_t *)&s_gpio0_input_state;
    gpio_monitor.output_state = (uint32_t *)&s_gpio0_output_state;
    gpio_monitor.pin_num = GPIO0_PIN_NUM;
    io_pin_monitor_gpio(&gpio_monitor);
#endif

#if defined(GPIO1_MONITOR_ENABLE)
    gpio_monitor_t gpio1_monitor;
    gpio1_monitor.port = GPIO1;
    gpio1_monitor.direction_state = (uint32_t *)&s_gpio1_direction_state;
    gpio1_monitor.pull_state = (uint32_t *)&s_gpio1_pull_state;
    gpio1_monitor.input_state = (uint32_t *)&s_gpio1_input_state;
    gpio1_monitor.output_state = (uint32_t *)&s_gpio1_output_state;
    gpio1_monitor.pin_num = GPIO1_PIN_NUM;
    io_pin_monitor_gpio(&gpio1_monitor);
#endif

#if defined(GPIO2_MONITOR_ENABLE)
    gpio_monitor_t gpio2_monitor;
    gpio2_monitor.port = GPIO2;
    gpio2_monitor.direction_state = (uint32_t *)&s_gpio2_direction_state;
    gpio2_monitor.pull_state = (uint32_t *)&s_gpio2_pull_state;
    gpio2_monitor.input_state = (uint32_t *)&s_gpio2_input_state;
    gpio2_monitor.output_state = (uint32_t *)&s_gpio2_output_state;
    gpio2_monitor.pin_num = GPIO2_PIN_NUM;
    io_pin_monitor_gpio(&gpio2_monitor);
#endif

#if defined(MSIO_MONITOR_ENABLE)
    for (uint8_t index = 0; index < MSIO_PIN_NUM; index++)
    {
#if defined(SOC_GR5515)
        s_msio_mode_state |= ((~(ll_msio_get_pin_mode(MSIO_PIN_0 << index)) & 0x1) << index);
        s_msio_direction_state |= ((ll_msio_get_pin_direction(MSIO_PIN_0 << index)) << (index * 2));
        s_msio_pull_state |= ((ll_msio_get_pin_pull(MSIO_PIN_0 << index)) << (index * 2));
    }
    s_msio_input_state = ll_msio_read_input_port();
    s_msio_output_state = ll_msio_read_output_port();
#else
#if defined(SOC_GR5526)
        s_msio_mode_state |= ((~(ll_msio_get_pin_mode(MSIOA, MSIO_PIN_0 << index)) & 0x1) << index);
#else
        s_msio_mode_state |= ((ll_msio_get_pin_mode(MSIOA, MSIO_PIN_0 << index)) << index);
#endif
        s_msio_direction_state |= ((ll_msio_get_pin_direction(MSIOA, MSIO_PIN_0 << index)) << (index * 2));
        s_msio_pull_state |= ((ll_msio_get_pin_pull(MSIOA, MSIO_PIN_0 << index)) << (index * 2));
    }
    s_msio_input_state = ll_msio_read_input_port(MSIOA);
    s_msio_output_state = ll_msio_read_output_port(MSIOA);
#endif
#endif

#if defined(AON_GPIO_MONITOR_ENABLE)
    for (uint8_t index = 0; index < AON_GPIO_PIN_NUM; index++)
    {
#if defined(SOC_GR5515) || defined(SOC_GR5526)
        uint32_t pin_direction = (ll_aon_gpio_get_pin_mode(AON_GPIO_PIN_0 << index) != LL_AON_GPIO_MODE_INPUT) ? 1 : 2;
        s_aon_gpio_direction_state |= ((pin_direction) << (index * 2));
        uint32_t pull_state = ll_aon_gpio_get_pin_pull(AON_GPIO_PIN_0 << index);
        switch (pull_state)
        {
            case LL_AON_GPIO_PULL_NO:
                pull_state = 0;
                break;
            case LL_AON_GPIO_PULL_UP:
                pull_state = 1;
                break;
            case LL_AON_GPIO_PULL_DOWN:
                pull_state = 2;
                break;
            default:
                pull_state = 3;
                break;
        }
        s_aon_gpio_pull_state |= (pull_state << (index * 2));
#else
        s_aon_gpio_direction_state |= ((ll_aon_gpio_get_pin_mode(AON_GPIO_PIN_0 << index)) << (index * 2));
        s_aon_gpio_pull_state |= ((ll_aon_gpio_get_pin_pull(AON_GPIO_PIN_0 << index)) << (index * 2));
#endif
    }
    s_aon_gpio_input_state = ll_aon_gpio_read_input_port();
    s_aon_gpio_output_state = ll_aon_gpio_read_output_port();
#endif
}

static pwr_mgmt_dev_state_t pwr_enter_sleep_check_io_pin_monitor(void)
{
    io_pin_monitor_schedule();
#if defined(SOC_GR533X) || defined(SOC_GR5X25) || defined(SOC_GR5526)
    return pwr_enter_sleep_check_new();
#else
    return pwr_enter_sleep_check();
#endif
}
