#include "bmi270.hpp"
#include "user_config.hpp"
#include "main.h"

namespace imu
{
namespace bmi270_protocol
{
uint32_t bias_checksum(const math::vector3& gyro, const math::vector3& accel);
} // namespace bmi270_protocol

namespace bmi270_service
{
std::atomic<bool> ready{false};
} // namespace bmi270_service

namespace
{
namespace cfg = user_config::ahrs;
using pnxtypes::status;

bmi270_control control{};
bmi270::calibration calibration{};
bool reset_requested = false;
uint32_t queued_ms = 0;
uint32_t last_tx_ms = 0;
uint8_t command_part = 0;

enum class action : uint8_t
{
    command,
    retry,
    reset,
    read_control,
    read_calibration,
    read_feedback
};

struct request
{
    action kind;
    bmi270::command operation;
    void* output;
    TX_SEMAPHORE completed{};
    status result = status::error;
};

static_assert(sizeof(request*) == sizeof(ULONG));
TX_QUEUE requests{};
ULONG request_storage[1]{};

// The caller owns its request until the worker signals completion.
status submit(action kind, bmi270::command operation = {}, void* output = nullptr)
{
    if (!bmi270_service::ready.load(std::memory_order_acquire))
    {
        return status::not_initialized;
    }
    request call{kind, operation, output};
    if (tx_semaphore_create(&call.completed, const_cast<CHAR*>("bmi270 request"), 0) != TX_SUCCESS)
    {
        return status::error;
    }
    auto pending = reinterpret_cast<ULONG>(&call);
    if (tx_queue_send(&requests, &pending, TX_WAIT_FOREVER) == TX_SUCCESS)
    {
        while (tx_semaphore_get(&call.completed, TX_WAIT_FOREVER) == TX_WAIT_ABORTED)
        {
            // The worker still holds the request after an aborted wait.
        }
    }
    (void)tx_semaphore_delete(&call.completed);
    return call.result;
}

enum class calibration_step : uint8_t
{
    idle,
    applying_offset,
    discovering,
    requesting,
    running
};

struct calibration_session
{
    calibration_step step = calibration_step::idle;
    uint8_t calibration_token = 0;
    uint32_t operation = 0;
    uint32_t started_ms = 0;
    uint32_t action_ms = 0;
    uint32_t status_sequence = 0;
};

calibration_session session{};

// Keep the operation/token and eligibility for a delayed ACK on transport retries.
void queue_send(uint32_t now_ms)
{
    command_part = 0;
    control.pending = true;
    control.acknowledged = false;
    control.transport = status::busy;
    control.code = bmi270::result::unknown;
    control.data = 0;
    control.sent_ms = 0;
    control.received_ms = 0;
    queued_ms = now_ms;
}

void queue_command(bmi270::command operation, uint32_t now_ms)
{
    control.operation = operation;
    ++control.token;
    if (operation == bmi270::command::calibrate)
    {
        session.calibration_token = control.token;
    }
    control.awaiting_ack = false;
    queue_send(now_ms);
}

void begin_calibration(uint32_t now_ms)
{
    session.step = calibration_step::discovering;
    control.already_completed = false;
    control.resumed_existing = false;
    session.started_ms = now_ms;
    session.action_ms = now_ms;
    session.status_sequence = calibration.sequence;
    control.startup = bmi270_startup::calibrating;
    queue_command(bmi270::command::read_last_calibration, now_ms);
}

void begin_offset(uint32_t now_ms)
{
    session.step = calibration_step::applying_offset;
    session.started_ms = now_ms;
    session.action_ms = now_ms;
    control.already_completed = false;
    control.resumed_existing = false;
    control.startup = bmi270_startup::configuring;
    queue_command(bmi270::command::set_offsets, now_ms);
}

void end_session(bmi270_startup outcome)
{
    session.step = calibration_step::idle;
    control.startup = outcome;
    control.pending = false;
    control.awaiting_ack = false;
}

// Final status can finish a session before its outstanding query ACK arrives.
bool advance_session(uint32_t now_ms)
{
    if (session.step == calibration_step::idle)
    {
        return false;
    }
    const bool terminal = calibration.state == bmi270::calibration_state::success ||
                          calibration.state == bmi270::calibration_state::failed;
    if (session.step == calibration_step::running && terminal &&
        calibration.sequence != session.status_sequence &&
        calibration.operation == session.operation)
    {
        const bool success = calibration.state == bmi270::calibration_state::success &&
                             calibration.code == bmi270::result::ok && calibration.valid;
        end_session(success ? bmi270_startup::complete : bmi270_startup::failed);
        control.transport = status::ok;
        control.code = calibration.code;
        control.data = calibration.operation;
        control.received_ms = calibration.received_ms;
        return true;
    }
    if (now_ms - session.started_ms >= cfg::bmi270_calibration_timeout_ms)
    {
        // Missing the final report does not prove that saving failed.
        end_session(bmi270_startup::timeout);
        control.transport = status::timeout;
        control.code = bmi270::result::unknown;
        return false;
    }
    if (session.step != calibration_step::running &&
        session.step != calibration_step::applying_offset && control.acknowledged &&
        control.code != bmi270::result::busy && control.code != bmi270::result::not_ready)
    {
        end_session(bmi270_startup::failed);
        return false;
    }
    if (control.pending || now_ms - session.action_ms < cfg::bmi270_calibration_poll_ms)
    {
        return false;
    }
    const bool replay =
        session.step == calibration_step::running &&
        (control.already_completed ||
            (control.awaiting_ack && control.operation == bmi270::command::read_status));
    if (replay)
    {
        // Query/final status may have been lost after the node went quiet.
        control.operation = bmi270::command::calibrate;
        control.token = session.calibration_token;
        control.awaiting_ack = false;
    }
    if (replay || session.step != calibration_step::running || control.awaiting_ack)
    {
        queue_send(now_ms);
    }
    else
    {
        queue_command(bmi270::command::read_status, now_ms);
    }
    session.action_ms = now_ms;
    return false;
}
} // namespace

status bmi270_service::initialize(uint32_t now_ms)
{
    control.node_id = bmi270::gyro_id;
    control.startup = bmi270_startup::waiting;
    last_tx_ms = now_ms - cfg::bmi270_command_interval_ms;
    return tx_queue_create(&requests, const_cast<CHAR*>("bmi270 requests"), 1, request_storage,
               sizeof(request_storage)) == TX_SUCCESS
        ? status::ok
        : status::error;
}

void bmi270_service::discard_queue()
{
    (void)tx_queue_delete(&requests);
}

bool bmi270_service::apply_reset()
{
    if (!reset_requested)
    {
        return false;
    }
    reset_requested = false;
    end_session(bmi270_startup::waiting);
    control.acknowledged = false;
    control.already_completed = false;
    control.resumed_existing = false;
    control.transport = status::error;
    control.code = bmi270::result::unknown;
    control.operation = bmi270::command::none;
    calibration = {};
    return true;
}

// Late ACKs remain eligible after a transport timeout.
bool bmi270_service::acknowledge(const bmi270::ack& ack, uint32_t now_ms)
{
    if (!control.awaiting_ack || ack.token != control.token || ack.operation != control.operation)
    {
        return false;
    }
    control.pending = false;
    control.awaiting_ack = false;
    control.acknowledged = true;
    control.transport = status::ok;
    control.code = ack.code;
    control.data = ack.data;
    control.received_ms = ack.received_ms;
    if (session.step == calibration_step::applying_offset)
    {
        if (ack.code == bmi270::result::ok &&
            ack.data == bmi270_protocol::bias_checksum(user_config::bmi270.gyro_offset,
                            user_config::bmi270.accel_offset))
        {
            end_session(bmi270_startup::complete);
            return true;
        }
        if (ack.code == bmi270::result::ok)
        {
            control.transport = status::error;
            control.code = bmi270::result::unknown;
        }
        return false;
    }
    if (session.step == calibration_step::idle)
    {
        return false;
    }
    if (session.step == calibration_step::discovering)
    {
        if (ack.code == bmi270::result::busy || ack.code == bmi270::result::not_ready)
        {
            return false;
        }
        const bool missing = ack.code == bmi270::result::not_found && ack.data == UINT32_MAX;
        if (!missing &&
            ((ack.code != bmi270::result::accepted && ack.code != bmi270::result::completed) ||
                (ack.data >> 16) != 0 ||
                ((ack.data >> 8) != 1 &&
                    (ack.data >> 8) != static_cast<uint8_t>(bmi270::command::calibrate))))
        {
            // An unknown discovery result must never create a new calibration.
            end_session(bmi270_startup::failed);
            control.transport = status::error;
            return false;
        }
        if (!missing && ack.code == bmi270::result::accepted && (ack.data >> 8) == 1)
        {
            // An older Flash-writing operation must finish before a RAM calibration starts.
            end_session(bmi270_startup::failed);
            control.transport = status::busy;
            return false;
        }
        session.step = calibration_step::requesting;
        if (!missing)
        {
            control.token = static_cast<uint8_t>(ack.data & 0xffU);
        }
        if (ack.code == bmi270::result::accepted)
        {
            // Recover the active request with its original token.
            session.calibration_token = control.token;
            control.resumed_existing = true;
            control.operation = bmi270::command::calibrate;
            queue_send(now_ms);
        }
        else
        {
            queue_command(bmi270::command::calibrate, now_ms);
        }
        return false;
    }
    if (ack.operation != bmi270::command::calibrate)
    {
        return false;
    }
    const bool accepted =
        ack.code == bmi270::result::accepted || ack.code == bmi270::result::completed;
    if (!accepted || (session.step == calibration_step::running && session.operation != ack.data))
    {
        if (session.step == calibration_step::running)
        {
            end_session(bmi270_startup::failed);
            control.transport = status::error;
            if (accepted)
            {
                // A changed operation makes the original request's outcome unknown.
                control.code = bmi270::result::unknown;
            }
        }
        return false;
    }
    session.step = calibration_step::running;
    session.operation = ack.data;
    control.already_completed = ack.code == bmi270::result::completed;
    control.resumed_existing = control.resumed_existing || control.already_completed;
    return false;
}

bool bmi270_service::report(const bmi270::calibration& received, uint32_t now_ms)
{
    calibration = received;
    return advance_session(now_ms);
}

bmi270_service::update bmi270_service::poll(uint32_t now_ms, bool can_ready)
{
    update result{};
    if (control.pending &&
        now_ms - (control.transport == status::busy ? queued_ms : control.sent_ms) >=
            cfg::bmi270_command_timeout_ms)
    {
        control.pending = false;
        control.transport = status::timeout;
    }
    if (control.startup == bmi270_startup::waiting && can_ready)
    {
        if (cfg::bmi270_calibrate_on_start)
        {
            begin_calibration(now_ms);
        }
        else
        {
            begin_offset(now_ms);
        }
    }
    result.finished = advance_session(now_ms);
    result.startup = control.startup;
    result.send = control.pending && control.transport == status::busy &&
                  now_ms - last_tx_ms >= cfg::bmi270_command_interval_ms;
    result.operation = control.operation;
    result.token = control.token;
    result.part = command_part;
    return result;
}

void bmi270_service::submitted(const update& request, status result, uint32_t now_ms)
{
    if (result == status::ok)
    {
        last_tx_ms = now_ms;
        if (request.operation == bmi270::command::set_offsets && request.part < 6)
        {
            command_part = request.part + 1;
            queued_ms = now_ms;
            return;
        }
        control.transport = status::ok;
        control.sent_ms = now_ms;
        control.awaiting_ack = true;
    }
    else
    {
        control.transport = result;
        control.pending = false;
    }
}

namespace
{
status request_command(bmi270::command operation)
{
    if (operation != bmi270::command::calibrate && operation != bmi270::command::read_status &&
        operation != bmi270::command::read_last_calibration &&
        operation != bmi270::command::set_offsets)
    {
        return status::invalid_arg;
    }
    if (control.pending || control.awaiting_ack || reset_requested ||
        control.startup == bmi270_startup::waiting)
    {
        return status::busy;
    }
    if (operation == bmi270::command::read_status && session.step != calibration_step::running)
    {
        return status::invalid_arg;
    }
    if (operation != bmi270::command::read_status && session.step != calibration_step::idle)
    {
        return status::busy;
    }

    const auto now_ms = HAL_GetTick();
    if (operation == bmi270::command::calibrate)
    {
        begin_calibration(now_ms);
    }
    else if (operation == bmi270::command::set_offsets)
    {
        begin_offset(now_ms);
    }
    else
    {
        queue_command(operation, now_ms);
        if (operation == bmi270::command::read_status)
        {
            session.action_ms = now_ms;
        }
    }
    return status::ok;
}

status retry_command()
{
    if (control.pending || reset_requested || control.startup == bmi270_startup::waiting)
    {
        return status::busy;
    }
    if (session.step != calibration_step::idle ||
        control.operation == bmi270::command::read_last_calibration)
    {
        const bool rejected =
            control.acknowledged &&
            (control.code == bmi270::result::busy || control.code == bmi270::result::not_ready);
        if (control.awaiting_ack || !control.acknowledged || rejected)
        {
            const auto now_ms = HAL_GetTick();
            // A rejected or unanswered request retains its token.
            queue_send(now_ms);
            session.action_ms = now_ms;
            return status::ok;
        }
    }
    return status::invalid_arg;
}
} // namespace

void bmi270_service::serve_request(const bmi270::sample& latest)
{
    ULONG pending = 0;
    if (tx_queue_receive(&requests, &pending, TX_NO_WAIT) != TX_SUCCESS)
    {
        return;
    }
    auto* call = reinterpret_cast<request*>(pending);
    switch (call->kind)
    {
        case action::command:
            call->result = request_command(call->operation);
            break;
        case action::retry:
            call->result = retry_command();
            break;
        case action::reset:
            reset_requested = true;
            call->result = status::ok;
            break;
        case action::read_control:
            *static_cast<bmi270_control*>(call->output) = control;
            call->result = status::ok;
            break;
        case action::read_calibration:
            if (calibration.version != 0)
            {
                *static_cast<bmi270::calibration*>(call->output) = calibration;
                call->result = status::ok;
            }
            break;
        case action::read_feedback:
            *static_cast<bmi270::sample*>(call->output) = latest;
            call->result = status::ok;
            break;
    }
    (void)tx_semaphore_put(&call->completed);
}

pnxtypes::status bmi270_request(bmi270::command operation)
{
    return submit(action::command, operation);
}

pnxtypes::status bmi270_retry()
{
    return submit(action::retry);
}

pnxtypes::status bmi270_reset()
{
    return submit(action::reset);
}

bool bmi270_read_control(bmi270_control& out)
{
    return submit(action::read_control, {}, &out) == status::ok;
}

bool bmi270_read_calibration(bmi270::calibration& out)
{
    return submit(action::read_calibration, {}, &out) == status::ok;
}

bool bmi270_read_feedback(bmi270::sample& out)
{
    if (submit(action::read_feedback, {}, &out) != status::ok)
    {
        return false;
    }
    const auto& config = user_config::bmi270;
    const auto now_ms = HAL_GetTick();
    out.gyro_valid = out.gyro_valid && now_ms - out.gyro_received_ms < config.gyro_timeout_ms;
    out.quaternion_valid =
        out.quaternion_valid && now_ms - out.quaternion_received_ms < config.quaternion_timeout_ms;
    out.accel_valid = out.accel_valid && now_ms - out.accel_received_ms < config.accel_timeout_ms;
    out.euler_valid = out.euler_valid && now_ms - out.euler_received_ms < config.euler_timeout_ms;
    return true;
}
} // namespace imu
