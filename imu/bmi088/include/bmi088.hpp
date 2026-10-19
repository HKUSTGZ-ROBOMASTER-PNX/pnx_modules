#pragma once

#include "status.hpp"
#include "geometry.hpp"
#include "config.hpp"

namespace imu
{

// Start the configured BMI088 acquisition/ESKF service from serialized ThreadX startup.
// Owns its sensor, SPI access and heater for firmware lifetime; do not use a second driver
// on the same hardware. Subscribe to imu::attitude::output for readiness and measurements.
pnxtypes::status start_bmi088();

// Wiring is supplied by the application; the driver never selects a board.
struct bmi088_config
{
    bsp::spi::handle spi;
    bsp::gpio::config acc_cs;
    bsp::gpio::config gyro_cs;
};

// Single owner thread for configure/calibrate/read/shutdown.
// No ISR access, internal worker, mutex or concurrent access to this instance.
class bmi088
{
public:
    explicit bmi088(const bmi088_config& wiring) : wiring(wiring)
    {
    }

    bmi088(const bmi088&) = delete;
    bmi088& operator=(const bmi088&) = delete;

    // Tune before sampling. Offsets are sensor-specific; defaults retain penix values.
    static constexpr uint32_t spi_timeout_ms = 2;
    static constexpr uint16_t calibration_samples = 4000;
    math::vector3 gyro_offset{gyro_preoffset_x, gyro_preoffset_y, gyro_preoffset_z};
    math::vector3 accel_offset{accel_preoffset_x, accel_preoffset_y, accel_preoffset_z};

    ~bmi088() = default;
    pnxtypes::status configure();

    struct calibration
    {
        uint16_t count = 0;
        pnxtypes::status result = pnxtypes::status::busy;
        math::vector3 mean{}, m2{};
        float accel_mean = 0.0f, accel_m2 = 0.0f;
    };

    // Feed fresh read_gyro/read_acc results once per sampling cycle, before additional
    // mounting/centripetal compensation. No I/O or delay. busy = collecting, ok/error = terminal
    // until progress = {}. Keep gyro_offset unchanged while collecting; only successful completion
    // replaces it.
    pnxtypes::status calibrate(calibration& progress, math::vector3 gyro, math::vector3 accel);
    pnxtypes::status shutdown();
    pnxtypes::status read_acc(math::vector3* data);
    pnxtypes::status read_gyro(math::vector3* data);
    pnxtypes::status read_temperature(float* data);

private:
    enum class cs : uint8_t
    {
        acc,
        gyro
    };
    const bmi088_config wiring;
    bool initialized = false;
    bool transport_ready = false;
    pnxtypes::status verify_chip_id(cs target);
    pnxtypes::status select_acc_spi();

    // len counts payload bytes; the accelerometer SPI dummy byte is discarded internally.
    pnxtypes::status read_reg(cs target, uint8_t addr, uint8_t* data, uint8_t len);
    pnxtypes::status write_reg(cs target, uint8_t addr, const uint8_t* data, uint8_t len);

    static constexpr uint8_t spi_write = 0x7F;
    static constexpr uint8_t spi_read = 0x80;

    static constexpr uint8_t acc_chip_id_addr = 0x00;
    static constexpr uint8_t acc_chip_id_val = 0x1E;
    static constexpr uint8_t acc_xyz_addr = 0x12;
    static constexpr uint8_t acc_xyz_len = 6;
    static constexpr uint8_t acc_range_6g = 0x01;
    static constexpr uint8_t acc_conf_addr = 0x40;
    static constexpr uint8_t acc_range_addr = 0x41;
    static constexpr uint8_t acc_int1_io_ctrl_addr = 0x53;
    static constexpr uint8_t acc_int_map_data_addr = 0x58;
    static constexpr uint8_t acc_pwr_conf_addr = 0x7C;
    static constexpr uint8_t acc_pwr_conf_act = 0x00;
    static constexpr uint8_t acc_pwr_ctrl_addr = 0x7D;
    static constexpr uint8_t acc_pwr_ctrl_on = 0x04;
    static constexpr uint8_t acc_soft_reset_addr = 0x7E;
    static constexpr uint8_t acc_soft_reset_val = 0xB6;

    static constexpr uint8_t gyro_chip_id_addr = 0x00;
    static constexpr uint8_t gyro_chip_id_val = 0x0F;
    static constexpr uint8_t gyro_xyz_addr = 0x02;
    static constexpr uint8_t gyro_xyz_len = 6;
    static constexpr uint8_t gyro_range_addr = 0x0F;
    static constexpr uint8_t gyro_range_1000 = 0x01;
    static constexpr uint8_t gyro_bandwidth_addr = 0x10;
    static constexpr uint8_t gyro_odr_2000_bw_230 = 0x01;
    static constexpr uint8_t gyro_lpm1_addr = 0x11;
    static constexpr uint8_t gyro_lpm1_normal = 0x00;
    static constexpr uint8_t gyro_lpm1_suspend = 0x80;
    static constexpr uint8_t gyro_int_ctrl_addr = 0x15;
    static constexpr uint8_t gyro_drdy_on = 0x80;
    static constexpr uint8_t gyro_int_io_conf_addr = 0x16;
    static constexpr uint8_t gyro_int_io_map_addr = 0x18;
    static constexpr uint8_t gyro_soft_reset_addr = 0x14;
    static constexpr uint8_t gyro_soft_reset_val = 0xB6;

    static constexpr uint8_t temp_addr = 0x22;
    static constexpr uint8_t temp_len = 2;
    static constexpr float temp_unit = 0.125f;
    static constexpr float temp_bias = 23.0f;

    static constexpr float accel_6g_sensitivity = 0.00179443359375f;
    static constexpr float gyro_1000_sensitivity = 0.0005326322180158476f;

    static constexpr float gyro_preoffset_x = -0.005280993487f;
    static constexpr float gyro_preoffset_y = -0.000237223741f;
    static constexpr float gyro_preoffset_z = -0.000647540528f;
    static constexpr float accel_preoffset_x = 0.0038458286072392397f;
    static constexpr float accel_preoffset_y = 0.00647039594993548f;
    static constexpr float accel_preoffset_z = 0.014968990490337293f;
};

} // namespace imu
