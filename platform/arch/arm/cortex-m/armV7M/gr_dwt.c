#include "gr_dwt.h"
#include "gr_common.h"

static uint32_t dwt_counter;

void hal_dwt_enable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial)
{
    GLOBAL_EXCEPTION_DISABLE();
    dwt_counter ++;
    CoreDebug->DEMCR = _demcr_initial | CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL = _dwt_ctrl_initial | DWT_CTRL_CYCCNTENA_Msk;
    //lint -e9036
    GLOBAL_EXCEPTION_ENABLE();
    return ;
}

void hal_dwt_disable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial)
{
    GLOBAL_EXCEPTION_DISABLE();
    dwt_counter --;
    if (dwt_counter == 0U)
    {
        DWT->CTRL = _dwt_ctrl_initial;
        CoreDebug->DEMCR = _demcr_initial;
    }
    //lint -e9036
    GLOBAL_EXCEPTION_ENABLE();
    return ;
}
