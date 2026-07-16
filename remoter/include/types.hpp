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
};

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
    uint16_t reserve : 16;
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

} // namespace remoter
