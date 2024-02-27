#ifndef __APP_DRV_H__
#define __APP_DRV_H__

#include "app_drv_config.h"

#if FLASH_PROTECT_PRIORITY
#define protection_push() platform_interrupt_protection_push()
#define protection_pop()  platform_interrupt_protection_pop()
#else
#define protection_push()
#define protection_pop()
#endif

#endif
