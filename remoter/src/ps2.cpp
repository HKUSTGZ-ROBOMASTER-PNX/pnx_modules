#include "remoter.hpp"

#include "memory.h"

#include <cmath>
#include <cstring>

namespace remoter
{
namespace
{

constexpr std::size_t ps2_dma_buffer_size = 32;
constexpr uint8_t frame_header = 0x0D;
constexpr uint8_t frame_tail = 0x0A;
constexpr uint8_t remote_disconnected_byte = 0xAB;
constexpr float axis_center = 127.5f;

alignas(32) uint8_t rx_buffer_storage[ps2_dma_buffer_size] RAM_D1_BSS{};

float normalize_axis(uint8_t raw)
{
    return (static_cast<float>(raw) - axis_center) / axis_center;
}

float apply_deadzone(float value, float deadzone)
{
    const float absolute = std::fabs(value);
    if (absolute <= deadzone)
    {
        return 0.0f;
    }

    const float magnitude = (absolute - deadzone) / (1.0f - deadzone);
    return std::copysign(magnitude, value);
}

void set_disconnected(ps2_state& raw, ps2_link_state link, ULONG now)
{
    raw.data = {};
    raw.data.offline = true;
    raw.data.active_source = source::ps2;
    raw.link = link;
    raw.buttons = 0;
    raw.pressed = 0;
    raw.released = 0;
    raw.left_x = 128;
    raw.left_y = 128;
    raw.right_x = 128;
    raw.right_y = 128;
    raw.last_update_tick = static_cast<uint32_t>(now);
}

void set_connected(ps2_state& raw, const ps2_uart_frame& frame, float deadzone, ULONG now)
{
    const uint16_t previous_buttons = raw.buttons;
    const uint16_t current_buttons =
        (static_cast<uint16_t>(frame.button_high) << 8U) |
        static_cast<uint16_t>(frame.button_low);

    state output{};
    output.offline = false;
    output.active_source = source::ps2;
    output.last_left_sw = raw.data.left_sw;
    output.last_right_sw = raw.data.right_sw;
    output.last_key = raw.data.key;
    output.left_x = apply_deadzone(normalize_axis(frame.left_x), deadzone);
    output.left_y = -apply_deadzone(normalize_axis(frame.left_y), deadzone);
    output.right_x = apply_deadzone(normalize_axis(frame.right_x), deadzone);
    output.right_y = -apply_deadzone(normalize_axis(frame.right_y), deadzone);

    output.key.W = is_held(current_buttons, ps2_button::up);
    output.key.S = is_held(current_buttons, ps2_button::down);
    output.key.A = is_held(current_buttons, ps2_button::left);
    output.key.D = is_held(current_buttons, ps2_button::right);
    output.fn_1 = is_held(current_buttons, ps2_button::l1);
    output.fn_2 = is_held(current_buttons, ps2_button::r1);
    output.button = is_held(current_buttons, ps2_button::cross);
    output.pause = is_held(current_buttons, ps2_button::start);

    raw.data = output;
    raw.link = ps2_link_state::connected;
    raw.buttons = current_buttons;
    raw.pressed = current_buttons & static_cast<uint16_t>(~previous_buttons);
    raw.released = previous_buttons & static_cast<uint16_t>(~current_buttons);
    raw.left_x = frame.left_x;
    raw.left_y = frame.left_y;
    raw.right_x = frame.right_x;
    raw.right_y = frame.right_y;
    raw.last_update_tick = static_cast<uint32_t>(now);
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
    set_disconnected(raw_msg, ps2_link_state::receiver_offline, tx_time_get());
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
                    set_disconnected(raw_msg, ps2_link_state::remote_disconnected, now);
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
            else if (byte == frame_header)
            {
                candidate_bytes[0] = byte;
                candidate_size = 1;
            }
            else
            {
                candidate_size = 0;
            }
        }

        if ((now - last_signal_tick) > self->cfg_.receiver_offline_timeout_ticks &&
            raw_msg.link != ps2_link_state::receiver_offline)
        {
            set_disconnected(raw_msg, ps2_link_state::receiver_offline, now);
            msg::publish(self->remoter_topic_, raw_msg);
        }
    }
}

} // namespace remoter
