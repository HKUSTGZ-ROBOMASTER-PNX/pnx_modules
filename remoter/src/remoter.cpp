#include "remoter.hpp"

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
                     ULONG now,
                     ULONG timeout_ticks)
{
    const bool dr16_online = source_online(dr16, now, timeout_ticks);
    const bool vt03_online = source_online(vt03, now, timeout_ticks);
    const bool ps2_online = source_online(ps2, now, timeout_ticks);

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
    remoter_topic_ = msg::create<state>();
    if (remoter_topic_ == nullptr)
    {
        return false;
    }

    dr16_sub_ = msg::subscribe<dr16_state>();
    vt03_sub_ = msg::subscribe<vt03_state>();
    ps2_sub_ = msg::subscribe<ps2_state>();
    if (!dr16_sub_.valid() || !vt03_sub_.valid() || !ps2_sub_.valid())
    {
        return false;
    }

    if (tx_thread_create(&thread_, const_cast<CHAR*>("remoter"), merge_thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         cfg_.thread_priority, cfg_.thread_priority, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    return true;
}

bool service::init(const config& cfg)
{
    if (initialized_)
    {
        return true;
    }
    cfg_ = cfg;
    if (msg::init() != types::status::ok)
    {
        return false;
    }

    bool any_source = false;
    if (::config::feature::enable_dr16 && !dr16::instance().init(cfg_.dr16))
    {
        return false;
    }
    any_source = any_source || ::config::feature::enable_dr16;

    if (::config::feature::enable_vt03 && !vt03::instance().init(cfg_.vt03))
    {
        return false;
    }
    any_source = any_source || ::config::feature::enable_vt03;

    if (::config::feature::enable_ps2 && !ps2::instance().init(cfg_.ps2))
    {
        return false;
    }
    any_source = any_source || ::config::feature::enable_ps2;

    if (!any_source || !create_resources())
    {
        return false;
    }

    initialized_ = true;
    return true;
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
            if (msg::read(self->ps2_sub_, ps2_msg) == types::status::ok)
            {
                ps2.data = ps2_msg.data;
                ps2.tick = tx_time_get();
                ps2.seen = true;
            }
        }

        const ULONG now = tx_time_get();
        active = choose_source(active, dr16, vt03, ps2, now,
                               self->cfg_.offline_timeout_ticks);

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
        default:
            output = {};
            output.offline = true;
            output.active_source = source::none;
            break;
        }

        output.ps2_link = ps2.data.ps2_link;
        output.ps2_buttons = ps2.data.ps2_buttons;
        output.ps2_pressed = ps2.data.ps2_pressed;
        output.ps2_released = ps2.data.ps2_released;
        output.ps2_event_count = ps2.data.ps2_event_count;

        output.update_count = ++update_count;
        msg::publish(self->remoter_topic_, output);
        output.last_left_sw = output.left_sw;
        output.last_right_sw = output.right_sw;
        output.last_key = output.key;
        tx_thread_sleep(1);
    }
}

} // namespace remoter
