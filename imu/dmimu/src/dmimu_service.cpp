#include "attitude.hpp"
#include "dmimu.hpp"
#include "user_config.hpp"
#include "main.h"
#include <atomic>
#include <cstring>

namespace imu
{
namespace
{
namespace cfg = user_config::ahrs;
using attitude::output;
using attitude::phase;
using attitude::source;
using attitude::state;
using pnxtypes::status;
imu::dmimu sensor;
TX_THREAD worker{};
TX_QUEUE rx_queue{};
TX_MUTEX* can_mutex = nullptr;
alignas(8) uint8_t stack[cfg::dmimu_stack_size]{};

struct rx_message
{
    ULONG received_ms;
    uint8_t data[8];
};

// ThreadX queue sizes are in ULONG words, not bytes.
static_assert(sizeof(rx_message) % sizeof(ULONG) == 0);
constexpr UINT message_words = sizeof(rx_message) / sizeof(ULONG);
ULONG rx_storage[cfg::dmimu_queue_frames * message_words]{};
std::atomic<uint32_t> dropped_frames{0};
static_assert(std::atomic<uint32_t>::is_always_lock_free);
bool attempted = false;
status initialization = status::not_initialized;

constexpr imu::dmimu::register_id requested_registers[]{
    imu::dmimu::register_id::accel_data,
    imu::dmimu::register_id::gyro_data,
    imu::dmimu::register_id::euler_data,
    imu::dmimu::register_id::quaternion_data,
};
constexpr uint8_t request_count = sizeof(requested_registers) / sizeof(requested_registers[0]);

void on_rx(bsp::can::handle bus, const bsp::can::rx_frame& frame)
{
    if (bus != user_config::dmimu.bus || frame.id != user_config::dmimu.master_id ||
        frame.len != 8 || frame.remote || frame.id_kind != bsp::can::id_type::standard ||
        frame.format != bsp::can::frame_mode::classic)
    {
        return;
    }
    rx_message received{};
    received.received_ms = HAL_GetTick();
    std::memcpy(received.data, frame.data, sizeof(received.data));
    if (tx_queue_send(&rx_queue, &received, TX_NO_WAIT) != TX_SUCCESS)
    {
        dropped_frames.fetch_add(1, std::memory_order_relaxed);
    }
}

void run(ULONG)
{
    state current{};
    current.device = source::dmimu;
    uint32_t last_request_ms = HAL_GetTick() - cfg::dmimu_request_period_ms;
    uint32_t handled_drops = 0;
    // An index at request_count means the previous request cycle is complete.
    uint8_t request_index = request_count;
    float previous_yaw = 0;
    bool have_yaw = false;
    for (;;)
    {
        const auto total_drops = dropped_frames.load(std::memory_order_relaxed);
        if (total_drops != handled_drops)
        {
            // Missing a component makes the current multi-frame sample unusable.
            handled_drops = total_drops;
            (void)tx_queue_flush(&rx_queue);
            sensor.clear();
            have_yaw = false;
        }
        current.dropped_frames = total_drops;

        // Bound each batch so ongoing CAN traffic cannot starve requests and publication.
        for (uint32_t count = 0; count < cfg::dmimu_queue_frames; ++count)
        {
            rx_message received{};
            if (tx_queue_receive(&rx_queue, &received, TX_NO_WAIT) != TX_SUCCESS)
            {
                break;
            }

            bsp::can::rx_frame frame{};
            frame.id = user_config::dmimu.master_id;
            frame.len = 8;
            std::memcpy(frame.data, received.data, sizeof(received.data));
            if (sensor.receive(user_config::dmimu.bus, frame, received.received_ms) != status::ok)
            {
                have_yaw = false;
                continue;
            }
            if (dropped_frames.load(std::memory_order_relaxed) != handled_drops)
            {
                sensor.clear();
                have_yaw = false;
                break;
            }

            imu::dmimu::sample sample{};
            if (!sensor.take_sample(sample, HAL_GetTick()))
            {
                continue;
            }

            current.quaternion = sample.quaternion;
            current.roll = sample.roll;
            current.pitch = sample.pitch;
            current.yaw = sample.yaw;
            if (!have_yaw)
            {
                current.total_yaw = current.yaw;
                have_yaw = true;
            }
            else
            {
                // Wrap only the increment; total_yaw can span multiple turns.
                current.total_yaw += std::remainder(current.yaw - previous_yaw, 2 * math::pi);
            }
            previous_yaw = current.yaw;
            current.gyro = sample.gyro;
            current.accel = sample.accel;
            current.temperature = sample.temperature;
            current.received_ms = sample.received_ms;
            ++current.sequence;
            current.result = status::ok;
        }

        const auto now_ms = HAL_GetTick();
        current.online = sensor.online(now_ms);
        current.stage = current.online ? phase::running : phase::offline;
        if (!current.online)
        {
            have_yaw = false;
            current.result = status::timeout;
        }

        if (user_config::dmimu.communication == imu::dmimu::mode::request)
        {
            if (request_index == request_count &&
                now_ms - last_request_ms >= cfg::dmimu_request_period_ms)
            {
                last_request_ms = now_ms;
                request_index = 0;
            }
            if (request_index < request_count)
            {
                // One request per worker pass; do not exhaust the three bxCAN TX mailboxes.
                auto sent = status::busy;
                if (tx_mutex_get(can_mutex, TX_NO_WAIT) == TX_SUCCESS)
                {
                    sent = sensor.read_register(requested_registers[request_index]);
                    (void)tx_mutex_put(can_mutex);
                }
                if (sent != status::ok)
                {
                    current.result = sent;
                }
                else
                {
                    ++request_index;
                }
            }
        }

        (void)output.publish(current);
        tx_thread_sleep(cfg::dmimu_wait_ticks);
    }
}
} // namespace

pnxtypes::status start_dmimu(TX_MUTEX* bus_mutex)
{
    if (attempted)
    {
        return initialization;
    }
    attempted = true;
    if (user_config::dmimu.communication == imu::dmimu::mode::request && bus_mutex == nullptr)
    {
        initialization = status::invalid_arg;
        return initialization;
    }
    can_mutex = bus_mutex;
    initialization = sensor.configure(user_config::dmimu);
    if (initialization != status::ok)
    {
        return initialization;
    }
    initialization = output.init();
    if (initialization != status::ok)
    {
        return initialization;
    }
    if (tx_queue_create(&rx_queue, const_cast<CHAR*>("dmimu rx"), message_words, rx_storage,
            sizeof(rx_storage)) != TX_SUCCESS)
    {
        initialization = status::error;
        return initialization;
    }
    if (tx_thread_create(&worker, const_cast<CHAR*>("ahrs dmimu"), run, 0, stack, sizeof(stack),
            cfg::thread_priority, cfg::thread_priority, TX_NO_TIME_SLICE,
            TX_DONT_START) != TX_SUCCESS)
    {
        (void)tx_queue_delete(&rx_queue);
        initialization = status::error;
        return initialization;
    }
    initialization = bsp::can::register_rx_callback(user_config::dmimu.bus,
        bsp::can::rx_callback::bind<&on_rx>());
    if (initialization != status::ok)
    {
        (void)tx_thread_delete(&worker);
        (void)tx_queue_delete(&rx_queue);
        return initialization;
    }
    // Registered callback and rx_queue retain static lifetime even if resume fails.
    // Never unregister other consumers on this shared CAN bus.
    initialization = tx_thread_resume(&worker) == TX_SUCCESS ? status::ok : status::error;
    return initialization;
}
} // namespace imu
