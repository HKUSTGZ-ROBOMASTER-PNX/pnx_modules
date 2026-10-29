#include "referee.hpp"

#include "crc.hpp"
#include "main.h"
#include "user_config.hpp"

#include <cstring>
#include <atomic>
#include <type_traits>

namespace referee
{

namespace
{
using pnxtypes::status;
config cfg{};
state published{};
TX_THREAD worker{};
alignas(8) uint8_t stack[user_config::referee_stack_size]{};
state current{};
callbacks handlers{};
bool attempted = false;
status initialization = status::not_initialized;
memory::buffer<256> rx_buffer RAM_D1_BSS{};

struct receive_fifo
{
    // One ISR producer and one thread consumer. Full buffers drop NEW bytes;
    // already accepted stream bytes retain their order. No allocation or wait.
    static constexpr uint32_t capacity = 256U;
    static_assert(std::atomic<uint32_t>::is_always_lock_free);

    void push(const uint8_t* data, uint16_t len, uint32_t received_ms)
    {
        if (data == nullptr)
        {
            return;
        }
        for (uint16_t i = 0; i < len; ++i)
        {
            const auto write = head_.load(std::memory_order_relaxed);
            if (write - tail_.load(std::memory_order_acquire) == capacity)
            {
                dropped_.fetch_add(1U, std::memory_order_relaxed);
                continue;
            }
            buffer_[write % capacity] = data[i];
            timestamps_[write % capacity] = received_ms;
            head_.store(write + 1U, std::memory_order_release);
        }
    }

    bool pop(uint8_t& byte, uint32_t& received_ms)
    {
        const auto read = tail_.load(std::memory_order_relaxed);
        if (read == head_.load(std::memory_order_acquire))
        {
            return false;
        }
        byte = buffer_[read % capacity];
        received_ms = timestamps_[read % capacity];
        tail_.store(read + 1U, std::memory_order_release);
        return true;
    }

    void clear()
    {
        tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    }

    uint32_t dropped_bytes() const
    {
        return dropped_.load(std::memory_order_relaxed);
    }

private:
    uint8_t buffer_[capacity]{};
    uint32_t timestamps_[capacity]{};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    std::atomic<uint32_t> dropped_{0};
};

receive_fifo fifo;

void on_rx(bsp::usart::handle, uint8_t* data, uint32_t size)
{
    fifo.push(data, static_cast<uint16_t>(size), HAL_GetTick());
}

template <typename T> struct cached_sample
{
    sample<T> value{};
    bool available = false;
};

template <typename T> cached_sample<T> cache{};

struct packet
{
    uint16_t command;
    const uint8_t* data;
    uint16_t size;
    uint32_t received_ms;
};

template <typename T> bool decode(const packet& frame, receive_callback<T> on_update)
{
    sample<T> value{};
    if constexpr (std::is_same_v<T, robot_interaction_data>)
    {
        if (frame.size < sizeof(RoboInteractData_t) || frame.size > protocol::max_data_length)
        {
            return false;
        }
        std::memcpy(&value.data.header, frame.data, sizeof(RoboInteractData_t));
        value.data.payload_length = frame.size - sizeof(RoboInteractData_t);
        std::memcpy(value.data.payload, frame.data + sizeof(RoboInteractData_t),
            value.data.payload_length);
    }
    else
    {
        static_assert(std::is_trivially_copyable_v<T>);
        if (frame.size != sizeof(T))
        {
            return false;
        }
        std::memcpy(&value.data, frame.data, sizeof(T));
    }
    value.received_ms = frame.received_ms;
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    value.sequence = cache<T>.value.sequence + 1;
    cache<T>.value = value;
    cache<T>.available = true;
    TX_RESTORE
    if (on_update)
    {
        on_update(value);
    }
    return true;
}

bool decode(const packet& frame)
{
    switch (frame.command)
    {
        case RefereeID::GameStatus:
            return decode<GameStatus_t>(frame, handlers.game_status);
        case RefereeID::GameResult:
            return decode<GameResult_t>(frame, handlers.game_result);
        case RefereeID::RobotHP:
            return decode<RobotHP_t>(frame, handlers.robot_hp);
        case RefereeID::EventData:
            return decode<EventData_t>(frame, handlers.event_data);
        case RefereeID::RefereeWarning:
            return decode<RefereeWarning_t>(frame, handlers.referee_warning);
        case RefereeID::DartInfo:
            return decode<DartInfo_t>(frame, handlers.dart_info);
        case RefereeID::GameRobotStatus:
            return decode<GameRobotStatus_t>(frame, handlers.game_robot_status);
        case RefereeID::PowerHeatData:
            return decode<PowerHeatData_t>(frame, handlers.power_heat_data);
        case RefereeID::GameRobotPos:
            return decode<GameRobotPos_t>(frame, handlers.game_robot_pos);
        case RefereeID::Buff:
            return decode<Buff_t>(frame, handlers.buff);
        case RefereeID::RobotHurt:
            return decode<RobotHurt_t>(frame, handlers.robot_hurt);
        case RefereeID::ShootData:
            return decode<ShootData_t>(frame, handlers.shoot_data);
        case RefereeID::ProjectileAllowance:
            return decode<BulletRemaining_t>(frame, handlers.projectile_allowance);
        case RefereeID::RfidStatus:
            return decode<RfidStatus_t>(frame, handlers.rfid_status);
        case RefereeID::DartClientCmd:
            return decode<DartClientCmd_t>(frame, handlers.dart_client_cmd);
        case RefereeID::GroundRobotPosition:
            return decode<GroundRobotPosition_t>(frame, handlers.ground_robot_position);
        case RefereeID::RadarMarkData:
            return decode<RadarMarkData_t>(frame, handlers.radar_mark_data);
        case RefereeID::SentryInfo:
            return decode<SentryInfo_t>(frame, handlers.sentry_info);
        case RefereeID::RadarInfo:
            return decode<RadarInfo_t>(frame, handlers.radar_info);
        case RefereeID::RoboInteractData:
            return decode<robot_interaction_data>(frame, handlers.robo_interact_data);
        default:
            return true; // No typed layout is defined for this command; do not dispatch it.
    }
}

// Owns only stream assembly state. A reset discards the partial frame, not cached data.
struct parser
{
    static constexpr uint16_t payload_offset = sizeof(FrameHeader) + sizeof(uint16_t);
    static constexpr uint16_t overhead = payload_offset + sizeof(uint16_t);
    uint8_t buffer[protocol::max_data_length + overhead]{};
    uint16_t used = 0;
    uint16_t expected = 0;
    uint32_t last_byte_ms = 0;

    void reset()
    {
        used = 0;
        expected = 0;
    }

    void expire(uint32_t now)
    {
        if (used != 0 && now - last_byte_ms >= cfg.offline_timeout_ms)
        {
            reset();
            ++current.frame_timeout_count;
        }
    }

    void push(uint8_t byte, uint32_t received_ms)
    {
        // Arrival timestamps prevent worker scheduling delays from expiring queued frames.
        expire(received_ms);
        last_byte_ms = received_ms;
        if (used == 0 && byte != 0xA5)
        {
            return;
        }
        buffer[used++] = byte;
        if (used == sizeof(FrameHeader))
        {
            if (crc::verify_crc8_checksum(buffer, sizeof(FrameHeader)) == 0)
            {
                ++current.crc8_error_count;
                reset();
                return;
            }
            const uint16_t length = buffer[1] | (buffer[2] << 8);
            if (length > protocol::max_data_length)
            {
                ++current.length_error_count;
                reset();
                return;
            }
            expected = length + overhead;
        }
        if (expected == 0 || used != expected)
        {
            return;
        }
        const uint16_t length = expected - overhead;
        const uint16_t command =
            buffer[sizeof(FrameHeader)] | (buffer[sizeof(FrameHeader) + 1] << 8);
        const bool valid = crc::verify_crc16_checksum(buffer, used) != 0;
        reset();
        if (!valid)
        {
            ++current.crc16_error_count;
            return;
        }
        if (!decode(packet{command, buffer + payload_offset, length, received_ms}))
        {
            ++current.length_error_count;
            return;
        }
        current.received_ms = received_ms;
        current.online = true;
        ++current.sequence;
    }
};

void run(ULONG)
{
    parser stream{};
    uint32_t dropped = 0;
    uint32_t last_recovery = HAL_GetTick();
    for (;;)
    {
        // Bound work per tick. Check overflow during draining as well as between batches:
        // bytes accepted after a gap must never complete a frame from before that gap.
        for (uint16_t processed = 0; processed < receive_fifo::capacity; ++processed)
        {
            const auto lost = fifo.dropped_bytes();
            if (lost != dropped)
            {
                dropped = lost;
                fifo.clear();
                stream.reset();
            }
            uint8_t byte = 0;
            uint32_t received_ms = 0;
            if (!fifo.pop(byte, received_ms))
            {
                break;
            }
            stream.push(byte, received_ms);
        }

        const auto now = HAL_GetTick();
        stream.expire(now);
        current.rx_dropped_bytes = fifo.dropped_bytes();
        current.updated_ms = now;
        if (now - current.received_ms >= cfg.offline_timeout_ms)
        {
            current.online = false;
        }
        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        published = current;
        TX_RESTORE

        if (bsp::usart::rx_status(cfg.uart.instance) != status::ok &&
            now - last_recovery >= cfg.offline_timeout_ms)
        {
            last_recovery = now;
            if (bsp::usart::stop_rx(cfg.uart.instance) == status::ok)
            {
                fifo.clear();
                stream.reset();
                (void)bsp::usart::start_rx_to_idle(cfg.uart.instance, rx_buffer,
                    bsp::usart::rx_callback::bind<&on_rx>());
            }
        }
        tx_thread_sleep(cfg.period_ticks);
    }
}

} // namespace

template <typename T> bool read(sample<T>& destination)
{
    static_assert(command_id<T> != 0xffff, "Unsupported referee packet type");
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    const bool valid = cache<T>.available;
    if (valid)
    {
        destination = cache<T>.value;
    }
    TX_RESTORE
    return valid;
}

template bool read(sample<GameStatus_t>&);
template bool read(sample<GameResult_t>&);
template bool read(sample<RobotHP_t>&);
template bool read(sample<EventData_t>&);
template bool read(sample<RefereeWarning_t>&);
template bool read(sample<DartInfo_t>&);
template bool read(sample<GameRobotStatus_t>&);
template bool read(sample<PowerHeatData_t>&);
template bool read(sample<GameRobotPos_t>&);
template bool read(sample<Buff_t>&);
template bool read(sample<RobotHurt_t>&);
template bool read(sample<ShootData_t>&);
template bool read(sample<BulletRemaining_t>&);
template bool read(sample<RfidStatus_t>&);
template bool read(sample<DartClientCmd_t>&);
template bool read(sample<GroundRobotPosition_t>&);
template bool read(sample<RadarMarkData_t>&);
template bool read(sample<SentryInfo_t>&);
template bool read(sample<RadarInfo_t>&);
template bool read(sample<robot_interaction_data>&);

state read()
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    const auto result = published;
    TX_RESTORE
    return result;
}

status init(const config& settings, const callbacks& on_update)
{
    if (attempted)
    {
        return initialization;
    }
    attempted = true;
    cfg = settings;
    handlers = on_update;
    initialization = status::busy;

    if (cfg.uart.instance == bsp::usart::handle::none)
    {
        return initialization = status::not_configured;
    }
    if (cfg.thread_priority >= TX_MAX_PRIORITIES || cfg.period_ticks == 0 ||
        cfg.period_ticks > INT32_MAX || cfg.offline_timeout_ms == 0 ||
        cfg.offline_timeout_ms > INT32_MAX)
    {
        return initialization = status::invalid_arg;
    }
    if (tx_thread_create(&worker, const_cast<CHAR*>("referee"), run, 0, stack, sizeof(stack),
            cfg.thread_priority, cfg.thread_priority, TX_NO_TIME_SLICE,
            TX_DONT_START) != TX_SUCCESS)
    {
        return initialization = status::error;
    }
    auto result = bsp::usart::init(cfg.uart, bsp::usart::mode::dma);
    bool owned = false;
    if (result == status::ok)
    {
        result = bsp::usart::start_rx_to_idle(cfg.uart.instance, rx_buffer,
            bsp::usart::rx_callback::bind<&on_rx>());
        owned = result == status::ok;
    }
    if (result == status::ok && tx_thread_resume(&worker) == TX_SUCCESS)
    {
        return initialization = status::ok;
    }
    if (result == status::ok)
    {
        result = status::error;
    }
    if (owned)
    {
        const auto stopped = bsp::usart::stop_rx(cfg.uart.instance);
        if (stopped != status::ok)
        {
            result = stopped;
        }
    }
    // Callbacks only access static storage, even if aborting DMA failed.
    (void)tx_thread_terminate(&worker);
    (void)tx_thread_delete(&worker);
    return initialization = result;
}

} // namespace referee
