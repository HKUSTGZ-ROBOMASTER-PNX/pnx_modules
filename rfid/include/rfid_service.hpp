#pragma once

#include "rfid.hpp"
#include "status.hpp"

namespace rfid
{
struct sample
{
    rfid::data data{};
    uint32_t received_ms = 0;
    uint32_t sequence = 0;
    bool online = false;
    pnxtypes::status last_status = pnxtypes::status::not_initialized;
    pnxtypes::status buzzer_status = pnxtypes::status::ok;
};

// Automatically called by the module startup hook. Serialized, idempotent startup.
// Owns UART RX/TX, buzzer timer, worker stack and buffers for firmware lifetime.
// Reader/card absence does not block startup; RX waits use deadlines in the worker.
pnxtypes::status init() noexcept;
// Thread-safe latest successful data; sequence == 0 means no successful read yet.
// Cached data survives link loss. Check online and received_ms (HAL milliseconds).
sample read() noexcept;
} // namespace rfid
