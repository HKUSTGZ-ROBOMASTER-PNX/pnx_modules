#pragma once

#include <cstdint>

namespace remoter
{

enum class sw_state : uint8_t
{
    low = 0,
    mid,
    up
};

enum class source : uint8_t
{
    none = 0,
    dr16,
    vt03,
    ps2,
};

enum class ps2_link_state : uint8_t
{
    connected = 0,
    remote_disconnected,
    receiver_offline,
};

enum class ps2_button : uint16_t
{
    square = 0x8000,
    cross = 0x4000,
    circle = 0x2000,
    triangle = 0x1000,

    r1 = 0x0800,
    l1 = 0x0400,
    r2 = 0x0200,
    l2 = 0x0100,

    left = 0x0080,
    down = 0x0040,
    right = 0x0020,
    up = 0x0010,

    start = 0x0008,
    r3 = 0x0004,
    l3 = 0x0002,
    select = 0x0001,
};

inline constexpr bool is_held(uint16_t buttons, ps2_button button)
{
    return (buttons & static_cast<uint16_t>(button)) != 0U;
}

struct key_state
{
    uint16_t W : 1;
    uint16_t S : 1;
    uint16_t A : 1;
    uint16_t D : 1;
    uint16_t SHIFT : 1;
    uint16_t CTRL : 1;
    uint16_t Q : 1;
    uint16_t E : 1;
    uint16_t R : 1;
    uint16_t F : 1;
    uint16_t G : 1;
    uint16_t Z : 1;
    uint16_t X : 1;
    uint16_t C : 1;
    uint16_t V : 1;
    uint16_t B : 1;
} __attribute__((packed));

struct state
{
    bool offline = true;
    source active_source = source::none;
    sw_state left_sw = sw_state::low;
    sw_state right_sw = sw_state::low;
    sw_state last_left_sw = sw_state::low;
    sw_state last_right_sw = sw_state::low;
    float right_x = 0.0f;
    float right_y = 0.0f;
    float left_x = 0.0f;
    float left_y = 0.0f;
    float wheel = 0.0f;
    float mouse_x = 0.0f;
    float mouse_y = 0.0f;
    float mouse_z = 0.0f;
    bool mouse_left = false;
    bool mouse_right = false;
    bool fn_1 = false;
    bool fn_2 = false;
    bool button = false;
    bool pause = false;
    key_state key{};
    key_state last_key{};
    uint32_t update_count = 0;
};

struct dr16_state
{
    state data{};
};

struct vt03_state
{
    state data{};
};

struct ps2_state
{
    state data{};
    ps2_link_state link = ps2_link_state::receiver_offline;

    uint16_t buttons = 0;
    uint16_t pressed = 0;
    uint16_t released = 0;

    uint8_t left_x = 128;
    uint8_t left_y = 128;
    uint8_t right_x = 128;
    uint8_t right_y = 128;

    uint32_t last_update_tick = 0;
};

inline sw_state map_switch(uint8_t raw)
{
    switch (raw)
    {
    case 1:
        return sw_state::up;
    case 2:
        return sw_state::low;
    case 3:
        return sw_state::mid;
    default:
        return sw_state::low;
    }
}

inline constexpr uint16_t rc_ch_value_offset = 1024;
inline constexpr uint16_t rc_ch_offset_max = 660;
inline constexpr uint8_t dr16_frame_size = 18;
inline constexpr uint8_t vt03_frame_size = 21;
inline constexpr uint8_t ps2_frame_size = 8;

struct dr16_frame
{
    uint16_t ch_0 : 11;
    uint16_t ch_1 : 11;
    uint16_t ch_2 : 11;
    uint16_t ch_3 : 11;
    uint16_t s1 : 2;
    uint16_t s2 : 2;

    int16_t mouse_x : 16;
    int16_t mouse_y : 16;
    int16_t mouse_z : 16;
    uint8_t mouse_left : 8;
    uint8_t mouse_right : 8;
    struct __attribute__((packed))
    {
        uint16_t W : 1;
        uint16_t S : 1;
        uint16_t A : 1;
        uint16_t D : 1;
        uint16_t SHIFT : 1;
        uint16_t CTRL : 1;
        uint16_t Q : 1;
        uint16_t E : 1;
        uint16_t R : 1;
        uint16_t F : 1;
        uint16_t G : 1;
        uint16_t Z : 1;
        uint16_t X : 1;
        uint16_t C : 1;
        uint16_t V : 1;
        uint16_t B : 1;
    } key;
    uint16_t wheel : 11;
    uint16_t reserve : 5;
} __attribute__((packed));

struct vt03_frame
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0 : 11;
    uint64_t ch_1 : 11;
    uint64_t ch_2 : 11;
    uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    struct __attribute__((packed))
    {
        uint16_t W : 1;
        uint16_t S : 1;
        uint16_t A : 1;
        uint16_t D : 1;
        uint16_t SHIFT : 1;
        uint16_t CTRL : 1;
        uint16_t Q : 1;
        uint16_t E : 1;
        uint16_t R : 1;
        uint16_t F : 1;
        uint16_t G : 1;
        uint16_t Z : 1;
        uint16_t X : 1;
        uint16_t C : 1;
        uint16_t V : 1;
        uint16_t B : 1;
    } key;
    uint16_t crc16;
} __attribute__((packed));

struct ps2_uart_frame
{
    uint8_t header;
    uint8_t button_high;
    uint8_t button_low;
    uint8_t left_y;
    uint8_t left_x;
    uint8_t right_y;
    uint8_t right_x;
    uint8_t tail;
};

static_assert(sizeof(ps2_uart_frame) == ps2_frame_size);

} // namespace remoter
