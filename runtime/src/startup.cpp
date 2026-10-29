#include "bsp_threadx.hpp"
#include "rfid_service.hpp"
#include "main.h"

extern "C" __attribute__((weak)) void app_start()
{
}

extern "C" void tx_application_define(void*)
{
    if (bsp::threadx::init() != pnxtypes::status::ok || rfid::init() != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }
    // RFID starts independently of application overrides of this hook.
    app_start();
}
