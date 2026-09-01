#pragma once

#include "bsp_usart.hpp"
#include "callback.hpp"
#include "config.hpp"
#include "msg.hpp"
#include "types.hpp"
#include "tx_api.h"

#include <cstddef>
#include <cstdint>

namespace remoter
{

// Called synchronously by the remoter merge thread after publishing state.
// The callback must not block. The state reference is valid only for this call.
using update_callback = core::callback<void(const state&)>;

struct dr16_config
{
    bsp::usart::port uart_port = app::uart::dr16;
    std::uint32_t thread_priority = 2;
    std::uint32_t rx_timeout_ticks = 100;
};

struct vt03_config
{
    bsp::usart::port uart_port = app::uart::vt03;
    std::uint32_t thread_priority = 2;
};

struct ps2_config
{
    bsp::usart::port uart_port = app::uart::ps2;
    std::uint32_t thread_priority = 2;
    std::uint32_t receiver_offline_timeout_ticks = 600;
    std::uint32_t frame_timeout_ticks = 20;
    float deadzone = 0.08f;
};

struct config
{
    dr16_config dr16{};
    vt03_config vt03{};
    ps2_config ps2{};
    update_callback on_update_callback{};
    std::uint32_t thread_priority = 2;
    std::uint32_t offline_timeout_ticks = 120;
};

class dr16
{
public:
    static dr16& instance();

    bool init(const dr16_config& cfg = {});
    TX_SEMAPHORE* rx_sem() { return &rx_sem_; }
    msg::channel<dr16_state>& output() noexcept { return output_; }

private:
    static void dr16_thread_entry(ULONG arg);
    void dr16_rx_callback(bsp::usart::port port, const bsp::usart::rx_frame& frame);
    bool create_resources();

    dr16_config cfg_{};
    TX_THREAD thread_{};
    TX_SEMAPHORE rx_sem_{};
    // F407 DR16 receive and publish paths leave insufficient headroom at 768 B.
    alignas(8) std::uint8_t stack_[1536]{};

    std::uint8_t* rx_buffer_ = nullptr;
    alignas(4) std::uint8_t frame_buffers_[2][dr16_frame_size]{};
    volatile std::uint8_t ready_frame_idx_ = 0;
    std::uint8_t write_frame_idx_ = 0;
    volatile bool frame_ready_ = false;
    msg::channel<dr16_state> output_{};
    bool initialized_ = false;
};

class vt03
{
public:
    static vt03& instance();

    bool init(const vt03_config& cfg = {});
    msg::channel<vt03_state>& output() noexcept { return output_; }

private:
    static void vt03_thread_entry(ULONG arg);
    void vt03_rx_callback(bsp::usart::port port, const bsp::usart::rx_frame& frame);
    bool create_resources();
    void fill_raw(state& raw, const vt03_frame& frame);

    vt03_config cfg_{};
    TX_THREAD thread_{};
    TX_SEMAPHORE rx_sem_{};
    alignas(8) std::uint8_t stack_[768]{};

    std::uint8_t* rx_buffer_ = nullptr;
    alignas(4) std::uint8_t frame_buffers_[2][vt03_frame_size]{};
    volatile std::uint8_t ready_frame_idx_ = 0;
    std::uint8_t write_frame_idx_ = 0;
    volatile bool frame_ready_ = false;
    msg::channel<vt03_state> output_{};
    bool initialized_ = false;
};

class ps2
{
public:
    static ps2& instance();

    bool init(const ps2_config& cfg = {});
    msg::channel<ps2_state>& output() noexcept { return output_; }

private:
    static constexpr std::size_t stream_buffer_size = 64;

    static void ps2_thread_entry(ULONG arg);
    void ps2_rx_callback(bsp::usart::port port, const bsp::usart::rx_frame& frame);
    bool create_resources();

    ps2_config cfg_{};
    TX_THREAD thread_{};
    TX_SEMAPHORE rx_sem_{};
    alignas(8) std::uint8_t stack_[768]{};

    std::uint8_t* rx_buffer_ = nullptr;
    std::uint8_t stream_buffer_[stream_buffer_size]{};
    volatile std::size_t stream_head_ = 0;
    volatile std::size_t stream_tail_ = 0;
    volatile bool stream_overflow_ = false;
    msg::channel<ps2_state> output_{};
    bool initialized_ = false;
};

class service
{
public:
    static service& instance();

    bool init(const config& cfg = {});
    msg::channel<state>& output() noexcept { return output_; }

private:
    static void merge_thread_entry(ULONG arg);
    bool create_resources();

    config cfg_{};
    TX_THREAD thread_{};
    // F407 source merging can overflow 768 B and corrupt the ThreadX control block.
    alignas(8) std::uint8_t stack_[1536]{};
    msg::channel<state> output_{};
    msg::subscriber dr16_sub_{};
    msg::subscriber vt03_sub_{};
    msg::subscriber ps2_sub_{};
    bool initialized_ = false;
};

} // namespace remoter
