#include "rfid_service.hpp"
#include "main.h"

// GNU ld redirects ThreadX's existing startup call here. The BSP retains its
// original USB initialization and weak application hook without depending on RFID.
extern "C" void __real_tx_application_define(void* memory);

extern "C" void __wrap_tx_application_define(void* memory)
{
    if (rfid::init() != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }
    // Threads are not scheduled yet; the real hook initializes USB and calls app_start().
    __real_tx_application_define(memory);
}
