#include "remoter.hpp"

#include "memory.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace remoter
{
namespace
{

constexpr std::size_t ps2_dma_buffer_size = 32;
constexpr std::size_t ps2_frame_size = 8;
constexpr uint8_t frame_header = 0x0D;
constexpr uint8_t frame_tail = 0x0A;
constexpr uint8_t remote_disconnected_byte = 0xAB;
constexpr uint8_t axis_x_center = 127;
constexpr uint8_t axis_y_center = 128;

alignas(32) uint8_t rx_buffer_storage[ps2_dma_buffer_size] RAM_D1_BSS{};

struct ps2_uart_frame
{
    uint8_t header;
    uint8_t button_high;
    uint8_t button_low;
    uint8_t left_y;
    uint8_t left_x;
    uint8_t right_y;
    uint8_t right_x;
    uint8_t tail;
};

static_assert(sizeof(ps2_uart_frame) == ps2_frame_size);

float normalize_axis(uint8_t raw, uint8_t center)
{
    if (raw >= center)
    {
        return static_cast<float>(raw - center) /
               static_cast<float>(UINT8_MAX - center);
    }
    return -static_cast<float>(center - raw) / static_cast<float>(center);
}

void apply_stick_deadzone(float& x, float& y, float deadzone)
{
    const float magnitude = std::max(std::fabs(x), std::fabs(y));
    if (magnitude <= deadzone)
    {
        x = 0.0f;
        y = 0.0f;
        return;
    }

    const float scaled_magnitude = (magnitude - deadzone) / (1.0f - deadzone);
    const float scale = scaled_magnitude / magnitude;
    x *= scale;
    y *= scale;
}

std::size_t retain_frame_suffix(uint8_t* bytes, std::size_t size)
{
    for (std::size_t i = size; i > 1U; --i)
    {
        const std::size_t index = i - 1U;
        if (bytes[index] == frame_header)
        {
            const std::size_t retained = size - index;
            std::memmove(bytes, bytes + index, retained);
            return retained;
        }
    }
    return 0;
}

void set_disconnected(ps2_state& message, ps2_link_state link, ULONG now,
                      bool signal_received)
{
    const uint16_t previous_buttons = message.data.ps2_buttons;
    const uint32_t previous_event_count = message.data.ps2_event_count;
    if (signal_received)
    {
        ++message.signal_count;
        message.last_signal_tick = static_cast<uint32_t>(now);
    }

    message.data = {};
    message.data.offline = true;
    message.data.active_source = source::ps2;
    message.data.ps2_link = link;
    message.data.ps2_released = previous_buttons;
    message.data.ps2_event_count =
        previous_event_count + (previous_buttons != 0U ? 1U : 0U);
    message.raw_left_x = axis_x_center;
    message.raw_left_y = axis_y_center;
    message.raw_right_x = axis_x_center;
    message.raw_right_y = axis_y_center;
}

void set_connected(ps2_state& message, const ps2_uart_frame& frame, float deadzone,
                   ULONG now)
{
    const uint16_t previous_buttons = message.data.ps2_buttons;
    const uint16_t current_buttons =
        (static_cast<uint16_t>(frame.button_high) << 8U) |
        static_cast<uint16_t>(frame.button_low);

    state output{};
    output.offline = false;
    output.active_source = source::ps2;
    output.ps2_link = ps2_link_state::connected;
    output.ps2_buttons = current_buttons;
    output.ps2_pressed = current_buttons & static_cast<uint16_t>(~previous_buttons);
    output.ps2_released = previous_buttons & static_cast<uint16_t>(~current_buttons);
    output.ps2_event_count = message.data.ps2_event_count;
    if (output.ps2_pressed != 0U || output.ps2_released != 0U)
    {
        ++output.ps2_event_count;
    }
    output.last_left_sw = message.data.left_sw;
    output.last_right_sw = message.data.right_sw;
    output.last_key = message.data.key;
    output.left_x = normalize_axis(frame.left_x, axis_x_center);
    output.left_y = -normalize_axis(frame.left_y, axis_y_center);
    output.right_x = normalize_axis(frame.right_x, axis_x_center);
    output.right_y = -normalize_axis(frame.right_y, axis_y_center);
    apply_stick_deadzone(output.left_x, output.left_y, deadzone);
    apply_stick_deadzone(output.right_x, output.right_y, deadzone);
    output.wheel = 0.0f;

    message.data = output;
    message.raw_left_x = frame.left_x;
    message.raw_left_y = frame.left_y;
    message.raw_right_x = frame.right_x;
    message.raw_right_y = frame.right_y;
    ++message.frame_count;
    ++message.signal_count;
    message.last_signal_tick = static_cast<uint32_t>(now);
}

} // namespace

ps2& ps2::instance()
{
    static ps2 inst;
    return inst;
}

bool ps2::create_resources()
{
    if (tx_semaphore_create(&rx_sem_, const_cast<CHAR*>("ps2_rx"), 0) != TX_SUCCESS)
    {
        return false;
    }

    remoter_topic_ = msg::create<ps2_state>();
    if (remoter_topic_ == nullptr)
    {
        return false;
    }

    std::memset(rx_buffer_storage, 0, sizeof(rx_buffer_storage));
    std::memset(stream_buffer_, 0, sizeof(stream_buffer_));
    rx_buffer_ = rx_buffer_storage;
    stream_head_ = 0;
    stream_tail_ = 0;
    stream_overflow_ = false;

    const bsp::usart::line_config line_config{
        9600,
        bsp::usart::word_length::bits_8,
        bsp::usart::stop_bits::one,
        bsp::usart::parity::none,
        true,
        true,
    };
    if (bsp::usart::configure(cfg_.uart_port, line_config) != types::status::ok)
    {
        return false;
    }
    if (bsp::usart::init(cfg_.uart_port, bsp::usart::mode::dma) != types::status::ok)
    {
        return false;
    }
    if (bsp::usart::start_rx_to_idle(cfg_.uart_port, rx_buffer_, ps2_dma_buffer_size,
                                     ps2_rx_callback, this, nullptr) != types::status::ok)
    {
        return false;
    }

    if (tx_thread_create(&thread_, const_cast<CHAR*>("ps2"), ps2_thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         cfg_.thread_priority, cfg_.thread_priority, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    return true;
}

bool ps2::init(const ps2_config& cfg)
{
    if (initialized_)
    {
        return true;
    }
    if (cfg.receiver_offline_timeout_ticks == 0U || cfg.frame_timeout_ticks == 0U ||
        cfg.deadzone < 0.0f || cfg.deadzone >= 1.0f)
    {
        return false;
    }

    cfg_ = cfg;
    if (msg::init() != types::status::ok)
    {
        return false;
    }
    if (!create_resources())
    {
        return false;
    }
    initialized_ = true;
    return true;
}

void ps2::ps2_rx_callback(bsp::usart::port, const bsp::usart::rx_frame& frame, void* user_data)
{
    auto* self = static_cast<ps2*>(user_data);
    if (self == nullptr || frame.data == nullptr || frame.len == 0U)
    {
        return;
    }

    for (std::size_t i = 0; i < frame.len; ++i)
    {
        const std::size_t next = (self->stream_head_ + 1U) % stream_buffer_size;
        if (next == self->stream_tail_)
        {
            self->stream_overflow_ = true;
            break;
        }
        self->stream_buffer_[self->stream_head_] = frame.data[i];
        self->stream_head_ = next;
    }
    tx_semaphore_put(&self->rx_sem_);
}

void ps2::ps2_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<ps2*>(arg);
    if (self == nullptr)
    {
        self = &instance();
    }

    ps2_state raw_msg{};
    set_disconnected(raw_msg, ps2_link_state::receiver_offline, tx_time_get(), false);
    msg::publish(self->remoter_topic_, raw_msg);

    ps2_uart_frame candidate{};
    std::size_t candidate_size = 0;
    ULONG candidate_tick = 0;
    ULONG last_signal_tick = tx_time_get();

    for (;;)
    {
        tx_semaphore_get(&self->rx_sem_, self->cfg_.frame_timeout_ticks);
        const ULONG now = tx_time_get();

        if (candidate_size != 0U &&
            (now - candidate_tick) > self->cfg_.frame_timeout_ticks)
        {
            candidate_size = 0;
        }

        uint8_t bytes[stream_buffer_size]{};
        std::size_t byte_count = 0;
        bool overflow = false;

        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        overflow = self->stream_overflow_;
        self->stream_overflow_ = false;
        while (self->stream_tail_ != self->stream_head_ && byte_count < stream_buffer_size)
        {
            bytes[byte_count++] = self->stream_buffer_[self->stream_tail_];
            self->stream_tail_ = (self->stream_tail_ + 1U) % stream_buffer_size;
        }
        TX_RESTORE

        if (overflow)
        {
            candidate_size = 0;
        }

        auto* candidate_bytes = reinterpret_cast<uint8_t*>(&candidate);
        for (std::size_t i = 0; i < byte_count; ++i)
        {
            const uint8_t byte = bytes[i];
            if (candidate_size == 0U)
            {
                if (byte == frame_header)
                {
                    candidate_bytes[0] = byte;
                    candidate_size = 1;
                    candidate_tick = now;
                }
                else if (byte == remote_disconnected_byte)
                {
                    set_disconnected(raw_msg, ps2_link_state::remote_disconnected, now, true);
                    last_signal_tick = now;
                    msg::publish(self->remoter_topic_, raw_msg);
                }
                continue;
            }

            candidate_bytes[candidate_size++] = byte;
            candidate_tick = now;
            if (candidate_size < ps2_frame_size)
            {
                continue;
            }

            if (candidate.header == frame_header && candidate.tail == frame_tail)
            {
                set_connected(raw_msg, candidate, self->cfg_.deadzone, now);
                last_signal_tick = now;
                msg::publish(self->remoter_topic_, raw_msg);
                candidate_size = 0;
            }
            else
            {
                candidate_size = retain_frame_suffix(candidate_bytes, candidate_size);
                if (candidate_size != 0U)
                {
                    candidate_tick = now;
                }
            }
        }

        if ((now - last_signal_tick) > self->cfg_.receiver_offline_timeout_ticks &&
            raw_msg.data.ps2_link != ps2_link_state::receiver_offline)
        {
            set_disconnected(raw_msg, ps2_link_state::receiver_offline, now, false);
            msg::publish(self->remoter_topic_, raw_msg);
        }
    }
}

} // namespace remoter
