#include "remoter.hpp"

#include "bsp_bindings.hpp"
#include "config.hpp"
#include "types.hpp"

namespace remoter
{
namespace
{

struct source_cache
{
    state data{};
    ULONG tick = 0;
    bool seen = false;
};

bool source_online(const source_cache& source, ULONG now, ULONG timeout_ticks)
{
    return source.seen && !source.data.offline && (now - source.tick) <= timeout_ticks;
}

source choose_source(source active,
                     const source_cache& dr16,
                     const source_cache& vt03,
                     const source_cache& ps2,
                     const source_cache& ps2_uart,
                     ULONG now,
                     ULONG timeout_ticks)
{
    const bool dr16_online = source_online(dr16, now, timeout_ticks);
    const bool vt03_online = source_online(vt03, now, timeout_ticks);
    const bool ps2_online = source_online(ps2, now, timeout_ticks);
    const bool ps2_uart_online = source_online(ps2_uart, now, timeout_ticks);

    if (active == source::dr16 && dr16_online)
    {
        return active;
    }
    if (active == source::vt03 && vt03_online)
    {
        return active;
    }
    if (active == source::ps2 && ps2_online)
    {
        return active;
    }
    if (active == source::ps2_uart && ps2_uart_online)
    {
        return active;
    }
    if (dr16_online)
    {
        return source::dr16;
    }
    if (vt03_online)
    {
        return source::vt03;
    }
    if (ps2_online)
    {
        return source::ps2;
    }
    if (ps2_uart_online)
    {
        return source::ps2_uart;
    }
    return source::none;
}

} // namespace

service& service::instance()
{
    static service inst;
    return inst;
}

bool service::create_resources()
{
    if (msg::init(output_) != types::status::ok)
    {
        return false;
    }

    if constexpr (::config::feature::enable_dr16)
    {
        dr16_sub_ = msg::subscribe(dr16::instance().output());
    }
    if constexpr (::config::feature::enable_vt03)
    {
        vt03_sub_ = msg::subscribe(vt03::instance().output());
    }
#if ENABLE_PS2
    ps2_sub_ = msg::subscribe(binding::ps2_instance().output());
#endif
    if constexpr (::config::feature::enable_ps2_uart)
    {
        ps2_uart_sub_ = msg::subscribe(ps2_uart::instance().output());
    }
    bool sources_ready =
        (!::config::feature::enable_dr16 || dr16_sub_.valid()) &&
        (!::config::feature::enable_vt03 || vt03_sub_.valid()) &&
        (!::config::feature::enable_ps2_uart || ps2_uart_sub_.valid());
#if ENABLE_PS2
    sources_ready = sources_ready && ps2_sub_.valid();
#endif
    if (!sources_ready)
    {
        return false;
    }

    if (tx_thread_create(&thread_, const_cast<CHAR*>("remoter"), merge_thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         params::remoter::thread_priority + 1U,
                         params::remoter::thread_priority + 1U, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    return true;
}

types::status service::init(const config& cfg)
{
    if (initialized_)
    {
        return types::status::ok;
    }
    cfg_ = cfg;
    const types::status message_status = msg::init();
    if (message_status != types::status::ok)
    {
        return message_status;
    }

    bool any_source = false;
    if constexpr (::config::feature::enable_dr16)
    {
        const types::status status = dr16::instance().init();
        if (status != types::status::ok)
        {
            return status;
        }
    }
    any_source = any_source || ::config::feature::enable_dr16;

    if constexpr (::config::feature::enable_vt03)
    {
        const types::status status = vt03::instance().init();
        if (status != types::status::ok)
        {
            return status;
        }
    }
    any_source = any_source || ::config::feature::enable_vt03;

#if ENABLE_PS2
    {
        const types::status status = binding::ps2_instance().init();
        if (status != types::status::ok)
        {
            return status;
        }
    }
#endif
    any_source = any_source || ::config::feature::enable_ps2;

    if constexpr (::config::feature::enable_ps2_uart)
    {
        const types::status status = ps2_uart::instance().init();
        if (status != types::status::ok)
        {
            return status;
        }
    }
    any_source = any_source || ::config::feature::enable_ps2_uart;

    if (!any_source)
    {
        return types::status::not_configured;
    }
    if (!create_resources())
    {
        return types::status::error;
    }

    initialized_ = true;
    return types::status::ok;
}

void service::merge_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<service*>(arg);
    if (self == nullptr)
    {
        self = &instance();
    }

    source_cache dr16{};
    source_cache vt03{};
    source_cache ps2{};
    source_cache ps2_uart{};
    state output{};
    source active = source::none;
    uint32_t update_count = 0;

    for (;;)
    {
        {
            dr16_state dr16_msg{};
            if (msg::read(self->dr16_sub_, dr16_msg) == types::status::ok)
            {
                dr16.data = dr16_msg.data;
                dr16.tick = tx_time_get();
                dr16.seen = true;
            }
        }

        {
            vt03_state vt03_msg{};
            if (msg::read(self->vt03_sub_, vt03_msg) == types::status::ok)
            {
                vt03.data = vt03_msg.data;
                vt03.tick = tx_time_get();
                vt03.seen = true;
            }
        }

        {
            ps2_state ps2_msg{};
            // Failed polls still publish diagnostics, but must not renew the link.
            if (msg::read(self->ps2_sub_, ps2_msg) == types::status::ok && ps2_msg.frame_valid)
            {
                ps2.data = ps2_msg.data;
                ps2.tick = ps2_msg.last_signal_tick;
                ps2.seen = true;
            }
        }

        {
            ps2_uart_state ps2_uart_msg{};
            if (msg::read(self->ps2_uart_sub_, ps2_uart_msg) == types::status::ok)
            {
                ps2_uart.data = ps2_uart_msg.data;
                ps2_uart.tick = tx_time_get();
                ps2_uart.seen = true;
            }
        }

        const ULONG now = tx_time_get();
        active = choose_source(active, dr16, vt03, ps2, ps2_uart, now,
                               params::remoter::offline_timeout_ticks);

        switch (active)
        {
        case source::dr16:
            output = dr16.data;
            output.active_source = active;
            output.offline = false;
            break;
        case source::vt03:
            output = vt03.data;
            output.active_source = active;
            output.offline = false;
            break;
        case source::ps2:
            output = ps2.data;
            output.active_source = active;
            output.offline = false;
            break;
        case source::ps2_uart:
            output = ps2_uart.data;
            output.active_source = active;
            output.offline = false;
            break;
        default:
            output = {};
            output.offline = true;
            output.active_source = source::none;
            break;
        }

        output.update_count = ++update_count;
        msg::publish(self->output_, output);
        if (self->cfg_.on_update_callback)
        {
            self->cfg_.on_update_callback(output);
        }
        output.last_left_sw = output.left_sw;
        output.last_right_sw = output.right_sw;
        output.last_key = output.key;
        tx_thread_sleep(1);
    }
}

} // namespace remoter
