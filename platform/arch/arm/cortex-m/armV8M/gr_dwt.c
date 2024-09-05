
#include "gr5x.h"
#include "gr_common.h"
#include "gr_dwt.h"

__IO uint32_t dwt_counter = 0x00;

void hal_dwt_enable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial)
{
    GLOBAL_EXCEPTION_DISABLE();
    dwt_counter ++;
    DCB->DEMCR = _demcr_initial | DCB_DEMCR_TRCENA_Msk;
    DWT->CTRL = _dwt_ctrl_initial | DWT_CTRL_CYCCNTENA_Msk;
    GLOBAL_EXCEPTION_ENABLE();
    return ;
}

void hal_dwt_disable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial)
{
    GLOBAL_EXCEPTION_DISABLE();
    dwt_counter --;
    if (dwt_counter == 0x0)
    {
        DWT->CTRL = _dwt_ctrl_initial;
        DCB->DEMCR = _demcr_initial;
    }
    GLOBAL_EXCEPTION_ENABLE();
}
