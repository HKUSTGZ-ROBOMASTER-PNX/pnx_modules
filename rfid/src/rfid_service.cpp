#include "rfid_service.hpp"
#include "remoter.hpp"
#include "buzzer.hpp"
#include "user_config.hpp"

namespace rfid
{
namespace
{
TX_THREAD thread{};
alignas(8) uint8_t stack[user_config::rfid::stack_size]{};
memory::buffer<64> dma_rx RAM_D1_BSS;
volatile uint8_t received[128]{};
volatile uint32_t received_size = 0;
volatile bool overflow = false;
sample current{};
sample cached{};
bool recognized = false;
bool attempted = false;
pnxtypes::status initialization = pnxtypes::status::not_initialized;

void publish() noexcept
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    cached = current;
    TX_RESTORE
}

uint8_t accept(const rfid::data& data) noexcept
{
    const auto value = data[0];
    if (value != 0x00 && value != 0x01)
    {
        return 0;
    }
    for (const auto byte : data)
    {
        if (byte != value)
        {
            return 0;
        }
    }
    const bool enabled = value == 0;
    if (recognized && remotes::enabled() == enabled)
    {
        return 0;
    }
    recognized = true;
    remotes::set_enabled(enabled);
    return enabled ? 2 : 1;
}

rfid::decoder decoder{user_config::rfid::address};
buzzer::player sound{user_config::rfid::buzzer};

void receive(bsp::usart::handle, uint8_t* bytes, uint32_t size)
{
    if (size > sizeof(received) - received_size)
    {
        overflow = true;
        return;
    }
    for (uint32_t i = 0; i < size; ++i)
    {
        received[received_size++] = bytes[i];
    }
}

void run(ULONG)
{
    using pnxtypes::status;
    const auto port = user_config::rfid_uart.instance;
    const bsp::usart::line_config line{user_config::rfid::baud};
    for (;;)
    {
        // configure() invalidates BSP state; select the transfer mode afterwards.
        auto result = bsp::usart::configure(port, line);
        if (result == status::ok)
        {
            // RX still uses DMA when the UART has no TX DMA channel (H723 USART10).
            const auto method =
                user_config::rfid_uart.dma.tx ? bsp::usart::mode::dma : bsp::usart::mode::block;
            result = bsp::usart::init(user_config::rfid_uart, method);
        }
        if (result == status::ok)
        {
            result = bsp::usart::start_rx_to_idle(port, dma_rx,
                bsp::usart::rx_callback::bind<receive>());
        }
        current.last_status = result;
        publish();
        if (result == status::ok)
        {
            break;
        }
        const auto failed_ms = HAL_GetTick();
        while (HAL_GetTick() - failed_ms < user_config::rfid::recovery_ms)
        {
            tx_thread_sleep(1);
        }
    }
    bool pending = false;
    uint32_t sent_ms = 0;
    uint32_t wait_ms = user_config::rfid::startup_ms;
    uint32_t idle_ms = HAL_GetTick();
    for (;;)
    {
        uint8_t bytes[sizeof(received)]{};
        uint32_t count = 0;
        bool lost = false;
        {
            TX_INTERRUPT_SAVE_AREA
            TX_DISABLE
            count = received_size;
            for (uint32_t i = 0; i < count; ++i)
            {
                bytes[i] = received[i];
            }
            lost = overflow;
            received_size = 0;
            overflow = false;
            TX_RESTORE
        }
        const uint32_t now = HAL_GetTick();
        const auto buzzer_result = sound.update(now);
        if (buzzer_result != status::ok)
        {
            current.buzzer_status = buzzer_result;
        }
        const bool rx_failed = bsp::usart::rx_status(port) != status::ok;
        if (lost || rx_failed || (pending && now - sent_ms >= user_config::rfid::timeout_ms))
        {
            current.online = false;
            // Re-arm the next presentation; losing the card never changes remote permission.
            recognized = false;
            current.last_status = rx_failed ? bsp::usart::rx_status(port) : status::timeout;
            pending = false;
            decoder.reset();
            count = 0;
            idle_ms = now;
            wait_ms = user_config::rfid::recovery_ms;
            if (rx_failed)
            {
                (void)bsp::usart::restart_rx(port);
            }
        }
        for (uint32_t i = 0; i < count; ++i)
        {
            rfid::data data{};
            const auto result = pending ? decoder.push(bytes[i], data) : decode_result::none;
            if (result == decode_result::failed)
            {
                pending = false;
                idle_ms = now;
                wait_ms = user_config::rfid::poll_ms;
                current.online = false;
                // Re-arm the next presentation; losing the card never changes remote permission.
                recognized = false;
                current.last_status = status::error;
            }
            else if (result == decode_result::success)
            {
                pending = false;
                idle_ms = now;
                wait_ms = user_config::rfid::poll_ms;
                current.data = data;
                current.received_ms = now;
                ++current.sequence;
                current.online = true;
                current.last_status = status::ok;
                const auto beeps = accept(data);
                if (beeps != 0)
                {
                    current.buzzer_status = sound.play(beeps, now);
                }
            }
        }
        // Protocol has no transaction ID. Require a quiet interval after a timeout.
        if (!pending && count != 0)
        {
            idle_ms = now;
        }
        if (!pending && !rx_failed && now - idle_ms >= wait_ms)
        {
            decoder.reset();
            const auto request =
                rfid::read_block(user_config::rfid::address, user_config::rfid::block);
            const uint32_t transmit_ms =
                (80000U + user_config::rfid::baud - 1U) / user_config::rfid::baud + 2U;
            pending = bsp::usart::transmit(port, request.data(), request.size(), transmit_ms) ==
                      status::ok;
            if (!pending)
            {
                current.online = false;
                // Re-arm the next presentation; losing the card never changes remote permission.
                recognized = false;
                current.last_status = status::error;
            }
            sent_ms = HAL_GetTick();
            idle_ms = sent_ms;
            wait_ms = user_config::rfid::recovery_ms;
        }
        publish();
        tx_thread_sleep(1);
    }
}
} // namespace

sample read() noexcept
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    const auto result = cached;
    TX_RESTORE
    return result;
}

pnxtypes::status init() noexcept
{
    if (attempted)
    {
        return initialization;
    }
    attempted = true;
    const auto created = tx_thread_create(&thread, const_cast<CHAR*>("rfid"), run, 0, stack,
        sizeof(stack), user_config::rfid::priority, user_config::rfid::priority, TX_NO_TIME_SLICE,
        TX_AUTO_START);
    initialization = created == TX_SUCCESS ? pnxtypes::status::ok : pnxtypes::status::error;
    return initialization;
}
} // namespace rfid
