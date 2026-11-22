#include "bmi270.hpp"
#include "user_config.hpp"
#include "main.h"
#include <atomic>
#include <cmath>
#include <cstring>

namespace imu
{
namespace
{
namespace cfg = user_config::ahrs;
using pnxtypes::status;
namespace control = bmi270_service;

bmi270 sensor;
TX_THREAD worker{};
TX_QUEUE rx_queue{};
attitude::output* output = nullptr;
alignas(8) uint8_t stack[cfg::bmi270_stack_size]{};

struct rx_message
{
    ULONG received_ms;
    uint32_t id;
    uint8_t data[8];
};

static_assert(sizeof(rx_message) % sizeof(ULONG) == 0);
constexpr UINT message_words = sizeof(rx_message) / sizeof(ULONG);
ULONG rx_storage[cfg::bmi270_queue_frames * message_words]{};
std::atomic<uint32_t> dropped_frames{0};
static_assert(std::atomic<uint32_t>::is_always_lock_free);
bool attempted = false;
status initialization = status::not_initialized;

void on_rx(bsp::can::handle, const bsp::can::rx_frame& frame)
{
    const auto& config = user_config::bmi270;
    const bool selected = (frame.id == bmi270::gyro_id && config.receive_gyro) ||
                          (frame.id == bmi270::quaternion_id && config.receive_quaternion) ||
                          (frame.id == bmi270::accel_id && config.receive_accel) ||
                          (frame.id == bmi270::euler_id && config.receive_euler) ||
                          frame.id == bmi270::reply_id;
    if (!selected || frame.len != 8 || frame.remote ||
        frame.id_kind != bsp::can::id_type::standard ||
        frame.format != bsp::can::frame_mode::classic)
    {
        return;
    }
    rx_message received{};
    received.received_ms = HAL_GetTick();
    received.id = frame.id;
    std::memcpy(received.data, frame.data, sizeof(received.data));
    if (tx_queue_send(&rx_queue, &received, TX_NO_WAIT) != TX_SUCCESS)
    {
        dropped_frames.fetch_add(1, std::memory_order_relaxed);
    }
}

void update_attitude(attitude::state& current, const bmi270::sample& latest, uint32_t now_ms,
    bmi270_startup startup)
{
    const auto& config = user_config::bmi270;
    current.gyro = latest.gyro;
    current.quaternion = latest.quaternion;
    current.accel = latest.accel;
    current.roll = config.receive_quaternion ? latest.roll : latest.euler.x;
    current.pitch = config.receive_quaternion ? latest.pitch : latest.euler.y;
    current.yaw = config.receive_quaternion ? latest.yaw : latest.euler.z;

    struct feedback_frame
    {
        bool enabled;
        bool valid;
        uint32_t received_ms;
    };

    const feedback_frame frames[]{
        {config.receive_gyro, latest.gyro_valid, latest.gyro_received_ms},
        {config.receive_quaternion, latest.quaternion_valid, latest.quaternion_received_ms},
        {config.receive_accel, latest.accel_valid, latest.accel_received_ms},
        {config.receive_euler, latest.euler_valid, latest.euler_received_ms},
    };
    bool selected = false;
    current.online = true;
    current.received_ms = now_ms;
    for (const auto& frame : frames)
    {
        if (!frame.enabled)
        {
            continue;
        }
        selected = true;
        current.online = current.online && frame.valid;
        // The combined attitude is only as fresh as its oldest selected frame.
        if (now_ms - frame.received_ms > now_ms - current.received_ms)
        {
            current.received_ms = frame.received_ms;
        }
    }
    if (!selected)
    {
        current.online = false;
        current.received_ms = 0;
    }
    current.stage = current.online ? attitude::phase::running : attitude::phase::offline;
    current.result = current.online ? status::ok : status::timeout;
    if (startup == bmi270_startup::waiting || startup == bmi270_startup::configuring ||
        startup == bmi270_startup::calibrating)
    {
        current.online = false;
        current.stage = startup == bmi270_startup::calibrating ? attitude::phase::calibrating
                                                               : attitude::phase::starting;
        current.result = status::busy;
    }
    else if (startup == bmi270_startup::failed || startup == bmi270_startup::timeout)
    {
        current.online = false;
        current.stage = attitude::phase::fault;
        current.result = startup == bmi270_startup::timeout ? status::timeout : status::error;
    }
}

void run(ULONG)
{
    attitude::state current{};
    current.device = attitude::source::bmi270;
    const auto& config = user_config::bmi270;
    uint32_t handled_drops = 0;
    uint32_t handled_angles = 0;
    uint32_t published_updates[4]{};
    bool bus_ready = false;
    constexpr uint32_t stream_refresh_ms = 1000;
    uint32_t last_stream_ms = HAL_GetTick() - stream_refresh_ms;
    float previous_yaw = 0;
    bool have_yaw = false;

    for (;;)
    {
        const bool reset = control::apply_reset();

        const auto total_drops = dropped_frames.load(std::memory_order_relaxed);
        auto* can_errors = bsp::can::err_sem(config.bus);
        const bool can_error =
            can_errors != nullptr && tx_semaphore_get(can_errors, TX_NO_WAIT) == TX_SUCCESS;
        const bool can_ready = bsp::can::current_status(config.bus) == status::ok;
        const bool bus_stopped = bus_ready && !can_ready;
        bus_ready = can_ready;
        if (reset || total_drops != handled_drops || can_error || bus_stopped)
        {
            handled_drops = total_drops;
            (void)tx_queue_flush(&rx_queue);
            sensor.clear();
            have_yaw = false;
        }
        current.dropped_frames = total_drops;

        for (uint32_t count = 0; count < cfg::bmi270_queue_frames; ++count)
        {
            rx_message received{};
            if (tx_queue_receive(&rx_queue, &received, TX_NO_WAIT) != TX_SUCCESS)
            {
                break;
            }
            const auto previous = sensor.read(received.received_ms);
            if (!(config.receive_quaternion ? previous.quaternion_valid : previous.euler_valid))
            {
                have_yaw = false;
            }
            bsp::can::rx_frame frame{};
            frame.id = received.id;
            frame.len = 8;
            std::memcpy(frame.data, received.data, sizeof(received.data));
            (void)sensor.receive(config.bus, frame, received.received_ms);
            if (dropped_frames.load(std::memory_order_relaxed) != handled_drops ||
                (can_errors != nullptr && tx_semaphore_get(can_errors, TX_NO_WAIT) == TX_SUCCESS) ||
                bsp::can::current_status(config.bus) != status::ok)
            {
                (void)tx_queue_flush(&rx_queue);
                sensor.clear();
                have_yaw = false;
                break;
            }

            bmi270::ack ack{};
            if (sensor.take_ack(ack) && control::acknowledge(ack, HAL_GetTick()))
            {
                sensor.clear();
                have_yaw = false;
            }
            bmi270::calibration received_calibration{};
            if (sensor.take_calibration(received_calibration) &&
                control::report(received_calibration, HAL_GetTick()))
            {
                have_yaw = false;
            }

            const auto latest = sensor.read(HAL_GetTick());
            const bool angle_valid =
                config.receive_quaternion ? latest.quaternion_valid : latest.euler_valid;
            const auto angle_updates =
                config.receive_quaternion ? latest.quaternion_updates : latest.euler_updates;
            const float yaw = config.receive_quaternion ? latest.yaw : latest.euler.z;
            if (!angle_valid)
            {
                have_yaw = false;
            }
            else if (angle_updates != handled_angles)
            {
                if (have_yaw)
                {
                    current.total_yaw += std::remainder(yaw - previous_yaw, 2 * math::pi);
                }
                else
                {
                    current.total_yaw = yaw;
                }
                previous_yaw = yaw;
                have_yaw = true;
            }
            handled_angles = angle_updates;
        }

        const uint32_t now_ms = HAL_GetTick();
        const auto latest = sensor.read(now_ms);
        const auto command = control::poll(now_ms, can_ready);

        update_attitude(current, latest, now_ms, command.startup);
        if (command.finished ||
            !(config.receive_quaternion ? latest.quaternion_valid : latest.euler_valid))
        {
            have_yaw = false;
        }
        const uint32_t updates[]{latest.gyro_updates, latest.quaternion_updates,
            latest.accel_updates, latest.euler_updates};
        if (current.online && std::memcmp(updates, published_updates, sizeof(updates)) != 0)
        {
            ++current.sequence;
            std::memcpy(published_updates, updates, sizeof(updates));
        }

        const bool refresh_stream = now_ms - last_stream_ms >= stream_refresh_ms;
        if (can_ready && (refresh_stream || command.send))
        {
            bool sent_command = !refresh_stream;
            auto sent =
                sensor.send(refresh_stream ? bmi270::command::set_stream : command.operation,
                    refresh_stream ? 0 : command.token, refresh_stream ? 0 : command.part);
            if (refresh_stream && sent != status::ok && command.send)
            {
                sent_command = true;
                sent = sensor.send(command.operation, command.token, command.part);
            }
            if (!sent_command && sent == status::ok)
            {
                last_stream_ms = HAL_GetTick();
            }
            if (sent_command && sent != status::busy)
            {
                control::submitted(command, sent, HAL_GetTick());
            }
        }
        (void)output->publish(current);
        control::serve_request(latest);
        tx_thread_sleep(cfg::bmi270_wait_ticks);
    }
}
} // namespace

pnxtypes::status start_bmi270(attitude::output& target)
{
    if (attempted)
    {
        return initialization;
    }
    attempted = true;
    initialization = sensor.configure(user_config::bmi270);
    if (initialization != status::ok)
    {
        return initialization;
    }
    initialization = target.init();
    if (initialization != status::ok)
    {
        return initialization;
    }
    output = &target;
    if (tx_queue_create(&rx_queue, const_cast<CHAR*>("bmi270 rx"), message_words, rx_storage,
            sizeof(rx_storage)) != TX_SUCCESS)
    {
        return initialization = status::error;
    }
    if (tx_thread_create(&worker, const_cast<CHAR*>("ahrs bmi270"), run, 0, stack, sizeof(stack),
            cfg::thread_priority, cfg::thread_priority, TX_NO_TIME_SLICE,
            TX_DONT_START) != TX_SUCCESS)
    {
        (void)tx_queue_delete(&rx_queue);
        return initialization = status::error;
    }
    initialization = control::initialize(HAL_GetTick());
    if (initialization != status::ok)
    {
        (void)tx_thread_delete(&worker);
        (void)tx_queue_delete(&rx_queue);
        return initialization;
    }
    initialization = bsp::can::register_rx_callback(user_config::bmi270.bus,
        bsp::can::rx_callback::bind<&on_rx>());
    if (initialization != status::ok)
    {
        (void)tx_thread_delete(&worker);
        (void)tx_queue_delete(&rx_queue);
        control::discard_queue();
        return initialization;
    }
    initialization = tx_thread_resume(&worker) == TX_SUCCESS ? status::ok : status::error;
    // The callback and queue retain static lifetime even if resume fails.
    control::ready.store(initialization == status::ok, std::memory_order_release);
    return initialization;
}

} // namespace imu
