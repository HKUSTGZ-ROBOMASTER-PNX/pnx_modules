#include "remoter.hpp"

#include "memory.h"

#include <cstring>

namespace remoter
{

namespace
{

constexpr int lost_threshold = 100;
uint8_t rx_buffer_storage[vt03_frame_size] RAM_D1_BSS{};

bool update_online(bool got, uint32_t& lost_cnt)
{
    if (got)
    {
        lost_cnt = 0;
        return true;
    }
    ++lost_cnt;
    return lost_cnt < static_cast<uint32_t>(lost_threshold);
}

} // namespace

vt03& vt03::instance()
{
    static vt03 inst;
    return inst;
}

bool vt03::create_resources()
{
    if (tx_semaphore_create(&rx_sem_, const_cast<CHAR*>("vt03_rx"), 0) != TX_SUCCESS)
    {
        return false;
    }

    remoter_topic_ = msg::create<vt03_state>();
    if (remoter_topic_ == nullptr)
    {
        return false;
    }

    if (bsp::usart::init(cfg_.uart_port, bsp::usart::mode::dma) != types::status::ok)
    {
        return false;
    }
    std::memset(rx_buffer_storage, 0, sizeof(rx_buffer_storage));
    std::memset(frame_buffers_, 0, sizeof(frame_buffers_));
    ready_frame_idx_ = 0;
    write_frame_idx_ = 0;
    frame_ready_ = false;
    rx_buffer_ = rx_buffer_storage;
    if (bsp::usart::start_rx_to_idle(cfg_.uart_port, rx_buffer_, vt03_frame_size, vt03_rx_callback,
                                     this, nullptr) != types::status::ok)
    {
        return false;
    }

    if (tx_thread_create(&thread_, const_cast<CHAR*>("vt03"), vt03_thread_entry,
                         reinterpret_cast<ULONG>(this), stack_,
                         sizeof(stack_),
                         cfg_.thread_priority, cfg_.thread_priority, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    return true;
}

bool vt03::init(const vt03_config& cfg)
{
    if (initialized_)
    {
        return true;
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

void vt03::fill_raw(state& raw, const vt03_frame& frame)
{
    raw.left_sw = map_switch(static_cast<uint8_t>(frame.mode_sw));
    raw.right_x = (static_cast<float>(frame.ch_0) - static_cast<float>(rc_ch_value_offset)) /
                  static_cast<float>(rc_ch_offset_max);
    raw.right_y = (static_cast<float>(frame.ch_1) - static_cast<float>(rc_ch_value_offset)) /
                  static_cast<float>(rc_ch_offset_max);
    raw.left_x = (static_cast<float>(frame.ch_3) - static_cast<float>(rc_ch_value_offset)) /
                 static_cast<float>(rc_ch_offset_max);
    raw.left_y = (static_cast<float>(frame.ch_2) - static_cast<float>(rc_ch_value_offset)) /
                 static_cast<float>(rc_ch_offset_max);
    raw.mouse_x = static_cast<float>(frame.mouse_x);
    raw.mouse_y = static_cast<float>(frame.mouse_y);
    raw.mouse_z = static_cast<float>(frame.mouse_z);
    raw.mouse_left = frame.mouse_left != 0;
    raw.mouse_right = frame.mouse_right != 0;
    raw.fn_1 = frame.fn_1 != 0;
    raw.fn_2 = frame.fn_2 != 0;
    raw.button = frame.trigger != 0;
    raw.pause = frame.pause != 0;
    std::memcpy(&raw.key, &frame.key, sizeof(raw.key));
}

void vt03::vt03_rx_callback(bsp::usart::port, const bsp::usart::rx_frame& frame, void* user_data)
{
    auto* self = static_cast<vt03*>(user_data);
    if (self != nullptr && frame.data != nullptr && frame.len == vt03_frame_size)
    {
        const uint8_t frame_idx = self->write_frame_idx_;
        std::memcpy(self->frame_buffers_[frame_idx], frame.data, vt03_frame_size);
        self->ready_frame_idx_ = frame_idx;
        self->write_frame_idx_ = static_cast<uint8_t>(frame_idx ^ 1U);
        self->frame_ready_ = true;
        tx_semaphore_put(&self->rx_sem_);
    }
}

void vt03::vt03_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<vt03*>(arg);
    if (self == nullptr)
    {
        self = &instance();
    }

    vt03_state raw_msg{};
    auto& output = raw_msg.data;
    state raw{};
    uint32_t lost_cnt = 0;
    output.offline = true;
    output.active_source = source::vt03;

    for (;;)
    {
        vt03_frame frame{};
        const bool got_frame = tx_semaphore_get(&self->rx_sem_, TX_NO_WAIT) == TX_SUCCESS;
        if (got_frame)
        {
            TX_INTERRUPT_SAVE_AREA
            TX_DISABLE
            const bool frame_ready = self->frame_ready_;
            if (frame_ready)
            {
                const uint8_t frame_idx = self->ready_frame_idx_;
                std::memcpy(&frame, self->frame_buffers_[frame_idx], sizeof(frame));
                self->frame_ready_ = false;
            }
            TX_RESTORE
        }

        const bool frame_ok = got_frame && frame.sof_1 == 0xA9 && frame.sof_2 == 0x53;
        const bool online = update_online(frame_ok, lost_cnt);

        if (frame_ok)
        {
            self->fill_raw(raw, frame);
        }

        if (online)
        {
            output.left_sw = raw.left_sw;
            output.right_x = raw.right_x;
            output.right_y = raw.right_y;
            output.left_x = raw.left_x;
            output.left_y = raw.left_y;
            output.mouse_x = raw.mouse_x;
            output.mouse_y = raw.mouse_y;
            output.mouse_z = raw.mouse_z;
            output.mouse_left = raw.mouse_left;
            output.mouse_right = raw.mouse_right;
            output.fn_1 = raw.fn_1;
            output.fn_2 = raw.fn_2;
            output.button = raw.button;
            output.pause = raw.pause;
            std::memcpy(&output.key, &raw.key, sizeof(output.key));
            output.offline = false;
        }
        else
        {
            output.offline = true;
        }

        msg::publish(self->remoter_topic_, raw_msg);
        output.last_left_sw = output.left_sw;
        std::memcpy(&output.last_key, &output.key, sizeof(output.last_key));
        tx_thread_sleep(1);
    }
}

} // namespace remoter
