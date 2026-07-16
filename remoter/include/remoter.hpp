#pragma once

#include "bsp_usart.hpp"
#include "config.hpp"
#include "msg.hpp"
#include "types.hpp"
#include "tx_api.h"

#include <cstdint>

namespace remoter
{

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

struct config
{
    dr16_config dr16{};
    vt03_config vt03{};
    std::uint32_t thread_priority = 2;
    std::uint32_t offline_timeout_ticks = 120;
};

class dr16
{
public:
    static dr16& instance();

    bool init(const dr16_config& cfg = {});
    TX_SEMAPHORE* rx_sem() { return &rx_sem_; }

private:
    static void dr16_thread_entry(ULONG arg);
    static void dr16_rx_callback(bsp::usart::port port, const bsp::usart::rx_frame& frame,
                                 void* user_data);
    bool create_resources();

    dr16_config cfg_{};
    TX_THREAD thread_{};
    TX_SEMAPHORE rx_sem_{};
    alignas(8) std::uint8_t stack_[768]{};

    std::uint8_t* rx_buffer_ = nullptr;
    alignas(4) std::uint8_t frame_buffers_[2][dr16_frame_size]{};
    volatile std::uint8_t ready_frame_idx_ = 0;
    std::uint8_t write_frame_idx_ = 0;
    volatile bool frame_ready_ = false;
    msg::topic* remoter_topic_ = nullptr;
    bool initialized_ = false;
};

class vt03
{
public:
    static vt03& instance();

    bool init(const vt03_config& cfg = {});

private:
    static void vt03_thread_entry(ULONG arg);
    static void vt03_rx_callback(bsp::usart::port port, const bsp::usart::rx_frame& frame,
                                 void* user_data);
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
    msg::topic* remoter_topic_ = nullptr;
    bool initialized_ = false;
};

class service
{
public:
    static service& instance();

    bool init(const config& cfg = {});

private:
    static void merge_thread_entry(ULONG arg);
    bool create_resources();

    config cfg_{};
    TX_THREAD thread_{};
    alignas(8) std::uint8_t stack_[768]{};
    msg::topic* remoter_topic_ = nullptr;
    msg::subscriber dr16_sub_{};
    msg::subscriber vt03_sub_{};
    bool initialized_ = false;
};

} // namespace remoter
