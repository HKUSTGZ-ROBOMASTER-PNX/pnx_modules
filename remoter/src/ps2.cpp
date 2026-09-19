#include "ps2.hpp"

#include "bsp_dwt.hpp"

namespace remoter
{
namespace
{

constexpr std::uint32_t spi_timeout_ms = 20U;
constexpr std::uint32_t gpio_half_period_us = 5U;
constexpr std::uint32_t gpio_cs_setup_us = 20U;
constexpr std::uint32_t gpio_cs_hold_us = 20U;
constexpr float axis_deadzone = 0.02f;
constexpr ULONG poll_period_ticks = 10U;

constexpr bool pressed(std::uint8_t buttons, std::uint8_t bit)
{
    return (buttons & static_cast<std::uint8_t>(1U << bit)) == 0U;
}

constexpr bool analog_id(std::uint8_t id)
{
    return id == 0x53U || id == 0x73U;
}

constexpr float normalize_axis(std::uint8_t raw, std::uint8_t center)
{
    if (raw >= center)
    {
        return static_cast<float>(raw - center) / static_cast<float>(UINT8_MAX - center);
    }
    return -static_cast<float>(center - raw) / static_cast<float>(center);
}

constexpr float apply_axis_deadzone(float value)
{
    return value > -axis_deadzone && value < axis_deadzone ? 0.0f : value;
}

uint16_t buttons_from(const ps2_controller_state& input)
{
    uint16_t buttons = 0U;
    const auto set = [&buttons](bool held, ps2_button button) {
        if (held)
        {
            buttons |= static_cast<uint16_t>(button);
        }
    };
    set(input.select, ps2_button::select);
    set(input.l3, ps2_button::l3);
    set(input.r3, ps2_button::r3);
    set(input.start, ps2_button::start);
    set(input.up, ps2_button::up);
    set(input.right, ps2_button::right);
    set(input.down, ps2_button::down);
    set(input.left, ps2_button::left);
    set(input.l2, ps2_button::l2);
    set(input.r2, ps2_button::r2);
    set(input.l1, ps2_button::l1);
    set(input.r1, ps2_button::r1);
    set(input.triangle, ps2_button::triangle);
    set(input.circle, ps2_button::circle);
    set(input.cross, ps2_button::cross);
    set(input.square, ps2_button::square);
    return buttons;
}

class transport_ref
{
public:
    transport_ref(void* context,
                  types::status (*transfer)(void*, const std::uint8_t*, std::uint8_t*, std::size_t))
        : context_(context), transfer_(transfer)
    {
    }

    types::status transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t size)
    {
        last_status = transfer_(context_, tx, rx, size);
        return last_status;
    }

    types::status last_status = types::status::not_initialized;

private:
    void* context_;
    types::status (*transfer_)(void*, const std::uint8_t*, std::uint8_t*, std::size_t);
};

} // namespace

types::status ps2_protocol::decode(const ps2_raw_frame& raw, ps2_controller_state& out) noexcept
{
    if (raw.bytes[0] != 0xFFU || !analog_id(raw.bytes[1]) || raw.bytes[2] != 0x5AU)
    {
        return types::status::error;
    }

    const std::uint8_t btn1 = raw.bytes[3];
    const std::uint8_t btn2 = raw.bytes[4];
    ps2_controller_state decoded{};
    decoded.select = pressed(btn1, 0U);
    decoded.l3 = pressed(btn1, 1U);
    decoded.r3 = pressed(btn1, 2U);
    decoded.start = pressed(btn1, 3U);
    decoded.up = pressed(btn1, 4U);
    decoded.right = pressed(btn1, 5U);
    decoded.down = pressed(btn1, 6U);
    decoded.left = pressed(btn1, 7U);
    decoded.l2 = pressed(btn2, 0U);
    decoded.r2 = pressed(btn2, 1U);
    decoded.l1 = pressed(btn2, 2U);
    decoded.r1 = pressed(btn2, 3U);
    decoded.triangle = pressed(btn2, 4U);
    decoded.circle = pressed(btn2, 5U);
    decoded.cross = pressed(btn2, 6U);
    decoded.square = pressed(btn2, 7U);
    decoded.rx = apply_axis_deadzone(normalize_axis(raw.bytes[5], 127U));
    decoded.ry = apply_axis_deadzone(-normalize_axis(raw.bytes[6], 128U));
    decoded.lx = apply_axis_deadzone(normalize_axis(raw.bytes[7], 127U));
    decoded.ly = apply_axis_deadzone(-normalize_axis(raw.bytes[8], 128U));
    out = decoded;
    return types::status::ok;
}

types::status ps2_spi::init()
{
    const types::status status = bsp::spi::init(spi_bus_);
    return status == types::status::ok ? bsp::gpio::set_active(cs_, false) : status;
}

types::status ps2_spi::transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t size)
{
    if (tx == nullptr || rx == nullptr || size == 0U)
    {
        return types::status::invalid_arg;
    }
    types::status status = bsp::gpio::set_active(cs_, true);
    if (status == types::status::ok)
    {
        status = bsp::spi::transmit_receive(spi_bus_, tx, rx, size, spi_timeout_ms);
    }
    const types::status release_status = bsp::gpio::set_active(cs_, false);
    return status == types::status::ok ? release_status : status;
}

types::status ps2_gpio::init()
{
    if (!bsp::dwt::initialized())
    {
        const types::status status = bsp::dwt::init();
        if (status != types::status::ok)
        {
            return status;
        }
    }
    types::status status = bsp::gpio::set_active(cs_, false);
    if (status == types::status::ok)
    {
        status = bsp::gpio::write(clk_, true);
    }
    return status == types::status::ok ? bsp::gpio::write(cmd_, true) : status;
}

types::status ps2_gpio::transfer_byte(std::uint8_t tx, std::uint8_t& rx)
{
    rx = 0U;
    for (std::uint8_t bit = 0U; bit < 8U; ++bit)
    {
        types::status status = bsp::gpio::write(cmd_, (tx & (1U << bit)) != 0U);
        if (status != types::status::ok)
        {
            return status;
        }
        bsp::dwt::delay_us(gpio_half_period_us);
        status = bsp::gpio::write(clk_, false);
        if (status != types::status::ok)
        {
            return status;
        }
        bsp::dwt::delay_us(gpio_half_period_us);
        bool data_high = false;
        status = bsp::gpio::read(data_, data_high);
        if (status != types::status::ok)
        {
            return status;
        }
        if (data_high)
        {
            rx = static_cast<std::uint8_t>(rx | (1U << bit));
        }
        status = bsp::gpio::write(clk_, true);
        if (status != types::status::ok)
        {
            return status;
        }
        bsp::dwt::delay_us(gpio_half_period_us);
    }
    return types::status::ok;
}

types::status ps2_gpio::transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t size)
{
    if (tx == nullptr || rx == nullptr || size == 0U)
    {
        return types::status::invalid_arg;
    }
    types::status status = bsp::gpio::set_active(cs_, true);
    if (status == types::status::ok)
    {
        bsp::dwt::delay_us(gpio_cs_setup_us);
        for (std::size_t index = 0U; index < size && status == types::status::ok; ++index)
        {
            status = transfer_byte(tx[index], rx[index]);
        }
        bsp::dwt::delay_us(gpio_cs_hold_us);
    }
    const types::status release_status = bsp::gpio::set_active(cs_, false);
    return status == types::status::ok ? release_status : status;
}

bool ps2::create_resources()
{
    if (msg::init(output_) != types::status::ok)
    {
        return false;
    }
    return tx_thread_create(&thread_, const_cast<CHAR*>("ps2"), poll_thread_entry,
                            reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                            cfg_.thread_priority, cfg_.thread_priority, TX_NO_TIME_SLICE,
                            TX_AUTO_START) == TX_SUCCESS;
}

types::status ps2::init(const ps2_config& cfg)
{
    if (initialized_)
    {
        return types::status::ok;
    }
    if (transport_ == nullptr || init_transport_ == nullptr || transfer_ == nullptr)
    {
        return types::status::not_configured;
    }
    cfg_ = cfg;
    const types::status message_status = msg::init();
    if (message_status != types::status::ok)
    {
        return message_status;
    }
    const types::status status = init_transport_(transport_);
    if (status != types::status::ok)
    {
        return status;
    }
    if (!create_resources())
    {
        return types::status::error;
    }
    initialized_ = true;
    return types::status::ok;
}

types::status ps2::poll_once(ps2_state& message)
{
    ps2_controller_state controller{};
    transport_ref transport{transport_, transfer_};
    const auto start_tick = tx_time_get();
    const types::status status = ps2_protocol::poll(transport, controller, &message.raw);
    message.poll_ticks = static_cast<std::uint32_t>(tx_time_get() - start_tick);
    message.transport_status = static_cast<std::uint8_t>(transport.last_status);
    // Diagnostic only: never accept a frame whose transport failed.
    ps2_controller_state decoded{};
    message.raw_decode_status = static_cast<std::uint8_t>(ps2_protocol::decode(message.raw, decoded));
    message.controller_id = message.raw.bytes[1];
    ++message.poll_count;
    if (status != types::status::ok)
    {
        message.frame_valid = false;
        ++message.error_count;
        return status;
    }

    const uint16_t previous_buttons = message.data.ps2_buttons;
    const uint16_t current_buttons = buttons_from(controller);
    state output{};
    output.offline = false;
    output.active_source = source::ps2;
    output.ps2_buttons = current_buttons;
    output.ps2_pressed = current_buttons & static_cast<uint16_t>(~previous_buttons);
    output.ps2_released = previous_buttons & static_cast<uint16_t>(~current_buttons);
    output.left_x = controller.lx;
    output.left_y = controller.ly;
    output.right_x = controller.rx;
    output.right_y = controller.ry;
    message.data = output;
    message.frame_valid = true;
    ++message.valid_frame_count;
    ++message.signal_count;
    message.last_signal_tick = static_cast<uint32_t>(tx_time_get());
    return types::status::ok;
}

void ps2::poll_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<ps2*>(arg);
    ps2_state message{};
    message.data.offline = true;
    message.data.active_source = source::ps2;
    for (;;)
    {
        self->poll_once(message);
        msg::publish(self->output_, message);
        tx_thread_sleep(poll_period_ticks);
    }
}

} // namespace remoter
