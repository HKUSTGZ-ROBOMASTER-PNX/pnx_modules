#include "remoter.hpp"

#include "bsp_dma.hpp"
#include "memory.h"

#include <cstring>

namespace remoter
{

namespace
{

bsp::dma::buffer<dr16_frame_size> rx_buffer_storage BSP_DMA_BUFFER{};

} // namespace

dr16& dr16::instance()
{
    static dr16 inst;
    return inst;
}

bool dr16::create_resources()
{
    if (tx_semaphore_create(&rx_sem_, const_cast<CHAR*>("dr16_rx"), 0) != TX_SUCCESS)
    {
        return false;
    }

    if (msg::init(output_, "remoter/dr16/raw") != types::status::ok)
    {
        return false;
    }

    std::memset(rx_buffer_storage.data(), 0, rx_buffer_storage.capacity());
    std::memset(frame_buffers_, 0, sizeof(frame_buffers_));
    ready_frame_idx_ = 0;
    write_frame_idx_ = 0;
    frame_ready_ = false;
    rx_buffer_ = rx_buffer_storage.data();

    if (bsp::usart::init(cfg_.uart_port, bsp::usart::mode::dma) != types::status::ok)
    {
        return false;
    }
    if (bsp::usart::start_rx_to_idle(cfg_.uart_port, rx_buffer_storage.view(),
                                     bsp::usart::rx_callback::bind<dr16, &dr16::dr16_rx_callback>(this),
                                     nullptr) != types::status::ok)
    {
        return false;
    }

    if (tx_thread_create(&thread_, const_cast<CHAR*>("dr16"), dr16_thread_entry,
                         reinterpret_cast<ULONG>(this), stack_,
                         sizeof(stack_), cfg_.thread_priority, cfg_.thread_priority,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    return true;
}

bool dr16::init(const dr16_config& cfg)
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

void dr16::dr16_rx_callback(bsp::usart::port, const bsp::usart::rx_frame& frame)
{
    if (frame.data != nullptr && frame.len == dr16_frame_size)
    {
        const uint8_t frame_idx = write_frame_idx_;
        std::memcpy(frame_buffers_[frame_idx], frame.data, dr16_frame_size);
        ready_frame_idx_ = frame_idx;
        write_frame_idx_ = static_cast<uint8_t>(frame_idx ^ 1U);
        frame_ready_ = true;
        tx_semaphore_put(&rx_sem_);
    }
}

void dr16::dr16_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<dr16*>(arg);
    if (self == nullptr)
    {
        self = &instance();
    }

    dr16_state raw_msg{};
    auto& msg = raw_msg.data;
    msg.offline = true;
    msg.active_source = source::dr16;

    for (;;)
    {
        while (tx_semaphore_get(&self->rx_sem_, self->cfg_.rx_timeout_ticks) != TX_SUCCESS)
        {
            msg.offline = true;
            bsp::usart::restart_rx(self->cfg_.uart_port);
            msg::publish(self->output_, raw_msg);
            tx_thread_sleep(3);
            bsp::usart::start_rx_to_idle(self->cfg_.uart_port, rx_buffer_storage.view(),
                                         bsp::usart::rx_callback::bind<dr16, &dr16::dr16_rx_callback>(self),
                                         nullptr);
        }

        TX_INTERRUPT_SAVE_AREA
        dr16_frame frame{};
        TX_DISABLE
        const bool frame_ready = self->frame_ready_;
        if (frame_ready)
        {
            const uint8_t frame_idx = self->ready_frame_idx_;
            std::memcpy(&frame, self->frame_buffers_[frame_idx], sizeof(frame));
            self->frame_ready_ = false;
        }
        TX_RESTORE

        if (!frame_ready)
        {
            continue;
        }

        msg.offline = false;
        msg.left_sw = map_switch(static_cast<uint8_t>(frame.s2));
        msg.right_sw = map_switch(static_cast<uint8_t>(frame.s1));

        msg.right_x = (static_cast<float>(frame.ch_0) - static_cast<float>(rc_ch_value_offset)) /
                        static_cast<float>(rc_ch_offset_max);
        msg.right_y = (static_cast<float>(frame.ch_1) - static_cast<float>(rc_ch_value_offset)) /
                        static_cast<float>(rc_ch_offset_max);
        msg.left_x = (static_cast<float>(frame.ch_2) - static_cast<float>(rc_ch_value_offset)) /
                       static_cast<float>(rc_ch_offset_max);
        msg.left_y = (static_cast<float>(frame.ch_3) - static_cast<float>(rc_ch_value_offset)) /
                       static_cast<float>(rc_ch_offset_max);
        msg.wheel = (static_cast<float>(frame.wheel) - static_cast<float>(rc_ch_value_offset)) /
                    static_cast<float>(rc_ch_offset_max);

        msg.mouse_x = static_cast<float>(frame.mouse_x);
        msg.mouse_y = static_cast<float>(frame.mouse_y);
        msg.mouse_z = static_cast<float>(frame.mouse_z);
        msg.mouse_left = frame.mouse_left != 0;
        msg.mouse_right = frame.mouse_right != 0;

        std::memcpy(&msg.key, &frame.key, sizeof(msg.key));
        msg::publish(self->output_, raw_msg);
        msg.last_left_sw = msg.left_sw;
        msg.last_right_sw = msg.right_sw;
        std::memcpy(&msg.last_key, &msg.key, sizeof(msg.last_key));
        tx_thread_sleep(1);
    }
}

} // namespace remoter
