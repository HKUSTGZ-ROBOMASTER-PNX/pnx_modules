#pragma once
#include "attitude.hpp"
#include "bsp_can.hpp"
#include "geometry.hpp"
#include <atomic>

namespace imu
{
// Decoder and transmitter have a single owner. receive() uses the ISR reception time
// in monotonic milliseconds; the caller owns CAN initialization and TX serialization.
class bmi270
{
public:
    static constexpr uint16_t command_id = 0x270;
    static constexpr uint16_t gyro_id = 0x271;
    static constexpr uint16_t quaternion_id = 0x272;
    static constexpr uint16_t accel_id = 0x273;
    static constexpr uint16_t euler_id = 0x274;
    static constexpr uint16_t reply_id = 0x275;

    enum class command : uint8_t
    {
        none = 0,
        calibrate = 6, // Calibrate the external node's active RAM bias; never save Flash.
        read_status = 2,
        read_last_calibration = 4,
        set_stream = 5,
        set_offsets = 0x16
    };

    enum class result : uint8_t
    {
        ok = 0,
        accepted = 1,
        busy = 2,
        invalid = 3,
        not_ready = 4,
        save_failed = 5,
        timeout = 6,
        motion = 7,
        sensor_error = 8,
        completed = 9,  // Duplicate request for an already finished operation.
        not_found = 10, // No accepted CAN calibration in this IMU power cycle.
        unknown = 0xff
    };

    enum class calibration_state : uint8_t
    {
        idle,
        warming,
        collecting,
        saving,
        success,
        failed
    };

    enum class phase : uint8_t
    {
        starting,
        warming,
        calibrating,
        running,
        offline,
        fault
    };

    struct config
    {
        bsp::can::handle bus = bsp::can::handle::none;
        uint16_t command_id = bmi270::command_id; // Host TX command ID.
        uint16_t target_id = gyro_id;             // Command payload target, also the gyro ID.
        uint32_t assembly_timeout_ms = 20;
        uint32_t gyro_timeout_ms = 5;
        uint32_t quaternion_timeout_ms = 10;
        uint32_t accel_timeout_ms = 5;
        uint32_t euler_timeout_ms = 10;
        bool receive_gyro = true;
        bool receive_quaternion = true;
        bool receive_accel = false;
        bool receive_euler = false;
        math::vector3 gyro_offset{};  // rad/s, raw fixed bias sent when startup calibration is off.
        math::vector3 accel_offset{}; // m/s^2, additive correction, includes no gravity removal.
    };

    struct sample
    {
        math::vector3 gyro{};               // rad/s, sensor axes
        math::quaternion quaternion{};      // NWU body-to-world, normalized WXYZ
        float roll = 0, pitch = 0, yaw = 0; // radians, no mounting transform
        math::vector3 accel{};              // m/s^2, sensor axes, includes gravity
        math::vector3 euler{};              // radians, x=roll, y=pitch, z=yaw
        uint32_t gyro_received_ms = 0;
        uint32_t quaternion_received_ms = 0;
        uint32_t accel_received_ms = 0;
        uint32_t euler_received_ms = 0;
        uint32_t gyro_updates = 0;
        uint32_t quaternion_updates = 0;
        uint32_t accel_updates = 0;
        uint32_t euler_updates = 0;
        uint8_t gyro_sequence = 0;
        uint8_t flags = 0;
        uint8_t accel_sequence = 0;
        uint8_t euler_sequence = 0;
        uint8_t accel_flags = 0;
        uint8_t euler_flags = 0;
        bool gyro_valid = false;
        bool quaternion_valid = false;
        bool accel_valid = false;
        bool euler_valid = false;
    };

    struct ack
    {
        command operation{};
        uint8_t token = 0;
        result code = result::unknown;
        uint32_t data = 0;
        uint32_t received_ms = 0;
    };

    struct calibration
    {
        uint16_t version = 0;
        uint16_t node_id = 0;
        uint32_t generation = 0; // External Flash generation, unchanged by RAM calibration.
        uint32_t operation = 0;
        uint32_t samples = 0;
        uint32_t required_samples = 0;
        uint32_t elapsed_ms = 0;
        math::vector3 gyro_offset{};  // Active fixed RAM bias, rad/s, subtracted before node ESKF.
        math::vector3 accel_offset{}; // Active RAM additive acceleration correction, m/s^2.
        float temperature = 0;        // Celsius, last node-reported offset temperature.
        bool valid = false;
        calibration_state state = calibration_state::idle;
        result code = result::unknown;
        phase stage = phase::starting;
        uint32_t sequence = 0;    // local completed-status count
        uint32_t received_ms = 0; // first fragment reception time
    };

    pnxtypes::status configure(const config& cfg);
    pnxtypes::status receive(bsp::can::handle bus, const bsp::can::rx_frame& frame,
        uint32_t received_ms);
    // Feedback timestamps and validity are independent; the frames do not share a sample ID.
    // Gyro BIAS_READY is diagnostic and is not required for valid feedback.
    sample read(uint32_t now_ms) const;
    bool take_ack(ack& out);
    bool take_calibration(calibration& out);
    // Discard all incomplete groups and invalidate attitude after loss or reboot.
    void clear();
    // TX submission only. The owner must match ACKs, serialize commands and retries.
    // set_stream sends the configured feedback mask without an ACK; its token is always zero.
    // set_offsets sends gyro XYZ, accel XYZ (parts 0..5), then CRC commit (part 6).
    // Only commit produces an ACK; both corrections become active together.
    pnxtypes::status send(command operation, uint8_t token, uint8_t part = 0);

private:
    config cfg_{};
    sample feedback_{};
    ack ack_{};
    calibration calibration_{};
    uint8_t status_bytes_[56]{};
    uint32_t status_started_ms_ = 0;
    uint32_t status_sequence_ = 0;
    uint8_t next_fragment_ = 0;
    uint8_t fragment_sequence_ = 0;
    bool configured_ = false;
    bool ack_available_ = false;
    bool calibration_available_ = false;
};

enum class bmi270_startup : uint8_t
{
    disabled,
    waiting,
    calibrating,
    complete,
    failed,
    timeout, // Final result unknown; the external RAM operation may have completed.
    configuring
};

struct bmi270_control
{
    uint16_t node_id = 0;
    bmi270_startup startup = bmi270_startup::disabled;
    bmi270::command operation{};
    uint8_t token = 0;
    bool pending = false;
    bool awaiting_ack = false; // Remains true after a submitted request times out.
    bool acknowledged = false;
    bool already_completed = false; // Device reports ended; terminal result may still be pending.
    bool resumed_existing = false;  // The discovered operation was already active or complete.
    pnxtypes::status transport = pnxtypes::status::not_initialized;
    bmi270::result code = bmi270::result::unknown;
    uint32_t data = 0;
    uint32_t sent_ms = 0;
    uint32_t received_ms = 0;
};

// Register before CAN initialization. The service sends control frames and consumes
// the bus error semaphore. The application owns CAN initialization and recovery.
// The output topic must remain alive for the firmware lifetime. Start calls must be
// serialized; the first result, output and mutex are retained. An optional mutex must
// remain alive for the firmware lifetime and be shared by all foreground CAN senders;
// sends never wait for it. Without a mutex, the caller must ensure TX serialization.
pnxtypes::status start_bmi270(attitude::output& output, TX_MUTEX* bus_mutex = nullptr);
// The following APIs run in application threads and wait for the BMI270 worker.
// Keep the calling thread alive until the call returns.
// One outstanding command. Calibrate discovers/reconciles the last
// operation before starting an explicitly requested RAM calibration. No Flash write is requested.
// read_status is accepted only during this service's active, acknowledged session.
// read_last_calibration is also available as a read-only query outside a session.
// set_stream is refreshed automatically by the service, not accepted by this entry point.
// set_offsets atomically reapplies both configured corrections to node RAM before fusion.
pnxtypes::status bmi270_request(bmi270::command operation);
// Retry the outstanding session request or last read-only discovery with the same token.
// Starting another calibration requires an explicit calibrate call.
pnxtypes::status bmi270_retry();
// Task context: discard pending groups after a known device reboot. This also
// invalidates cached calibration and repeats the selected startup policy after clearing RX state.
pnxtypes::status bmi270_reset();
bool bmi270_read_control(bmi270_control& out);
bool bmi270_read_calibration(bmi270::calibration& out);
// Latest independent feedback, including validity, flags and reception timestamps.
// Task context only; false until the service starts. This does not consume the cache.
bool bmi270_read_feedback(bmi270::sample& out);

// Internal interface between the BMI270 service and control implementation.
namespace bmi270_service
{
extern std::atomic<bool> ready;

struct update
{
    bmi270::command operation = bmi270::command::none;
    uint8_t token = 0;
    uint8_t part = 0;
    bmi270_startup startup = bmi270_startup::disabled;
    bool finished = false;
    bool send = false;
};

// Set up the request queue before starting the worker.
pnxtypes::status initialize(uint32_t now_ms);
void discard_queue();
// Runtime operations are called by the worker.
bool apply_reset();
bool acknowledge(const bmi270::ack& ack, uint32_t now_ms);
bool report(const bmi270::calibration& received, uint32_t now_ms);
update poll(uint32_t now_ms, bool can_ready);
void submitted(const update& request, pnxtypes::status result, uint32_t now_ms);
void serve_request(const bmi270::sample& latest);
} // namespace bmi270_service
} // namespace imu
