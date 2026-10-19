#pragma once
#include "bsp_can.hpp"
#include "geometry.hpp"

namespace imu
{
// Start the configured receive service before the application's shared CAN initialization.
// Request mode requires a firmware-lifetime mutex also used by every TX/lifecycle caller
// on that bus. The service never starts/stops CAN; subscribe to imu::attitude::output.
pnxtypes::status start_dmimu(TX_MUTEX* can_mutex = nullptr);

// Single owner thread. The application copies CAN frames in the ISR and supplies
// their reception timestamps in monotonic milliseconds (uint32_t wrap is supported).
class dmimu
{
public:
    enum class mode : uint8_t
    {
        request,
        active
    };
    enum class register_id : uint8_t
    {
        reboot = 0x00,
        accel_data = 0x01,
        gyro_data = 0x02,
        euler_data = 0x03,
        quaternion_data = 0x04,
        set_zero = 0x05,
        accel_calibration = 0x06,
        gyro_calibration = 0x07,
        magnetometer_calibration = 0x08,
        change_communication = 0x09,
        set_active_delay = 0x0A,
        change_active = 0x0B,
        set_baudrate = 0x0C,
        set_can_id = 0x0D,
        set_master_id = 0x0E,
        data_output_selection = 0x0F,
        save_parameters = 0xFE,
        restore_settings = 0xFF,
    };

    enum class ack_code : uint8_t
    {
        success = 0x00,
        register_not_found = 0x01,
        invalid_data = 0x02,
        operation_failed = 0x03,
        unknown = 0xFF,
    };

    struct config
    {
        bsp::can::handle bus = bsp::can::handle::none;
        uint32_t can_id = 1;
        uint32_t master_id = 17;
        mode communication = mode::active;
        uint32_t assembly_timeout_ms = 20;
        uint32_t offline_timeout_ms = 100;
    };

    struct sample
    {
        math::vector3 accel{};         // m/s^2, sensor axes
        math::vector3 gyro{};          // rad/s, sensor axes
        math::quaternion quaternion{}; // Normalized WXYZ, device convention; no mounting transform
        float roll = 0, pitch = 0, yaw = 0; // radians, device convention
        float temperature = 0;              // integer Celsius from acceleration frame
        uint32_t sequence = 0;
        uint32_t received_ms = 0; // oldest frame in this assembled result
    };

    struct reply
    {
        register_id reg{};
        ack_code ack = ack_code::unknown;
        uint32_t data = 0;
    };

    // Local configuration only: does not initialize CAN or change device settings.
    pnxtypes::status configure(const config& cfg);
    pnxtypes::status receive(bsp::can::handle bus, const bsp::can::rx_frame& frame,
        uint32_t received_ms);
    bool take_sample(sample& out, uint32_t now_ms);
    bool take_reply(reply& out);
    bool online(uint32_t now_ms) const;
    // Call after queue overflow, reboot or a change of device output configuration.
    void clear();
    // One command per call; return value is TX status, not the device ACK.
    pnxtypes::status read_register(register_id reg);
    pnxtypes::status write_register(register_id reg, uint32_t data);

private:
    config cfg_{};
    sample working_{};
    sample completed_{};
    reply reply_{};
    uint32_t sequence_ = 0;
    uint8_t pending_ = 0;
    bool configured_ = false;
    bool available_ = false;
    bool have_sample_ = false;
    bool reply_available_ = false;
    void decode_accel(const uint8_t data[8]);
    void decode_gyro(const uint8_t data[8]);
    void decode_euler(const uint8_t data[8]);
    void decode_quaternion(const uint8_t data[8]);
    pnxtypes::status send_command(register_id reg, bool write, uint32_t data);
};
} // namespace imu
