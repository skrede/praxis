#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_TRAPEZOID_OPTIONS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_TRAPEZOID_OPTIONS_H

#include "book_time_scalings.h"

#include "praxis/trajectory/time_scaling.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <optional>

namespace praxis::tests {

// The three options of Lynch & Park, Modern Robotics, sec. 9.2.2.2 -- choosing v and a, v and T, or
// a and T -- each solving the remaining quantity from s(T) = 1 by eq. (9.24) in one of the spellings a
// reader of that section arrives at, then sampling eq. (9.16)-(9.24) in one of the phase shapes below.

enum class ramp_test : std::uint8_t
{
    divided,
    multiplied
};

enum class domain_check : std::uint8_t
{
    none,
    printed
};

struct option_shape
{
    ramp_test ramp;
    deceleration late;
    end_handling ends;
    domain_check check;
};

inline constexpr std::size_t option_shape_count = 2u * 2u * 3u * 2u;

constexpr option_shape option_shape_at(std::size_t index)
{
    option_shape shape{};
    shape.check = static_cast<domain_check>(index % 2u);
    shape.ends  = static_cast<end_handling>(index / 2u % 3u);
    shape.late  = static_cast<deceleration>(index / 6u % 2u);
    shape.ramp  = static_cast<ramp_test>(index / 12u % 2u);

    return shape;
}

// T = (a + v^2)/(va) and its rearrangements.
inline constexpr std::size_t duration_spelling_count = 4u;

inline double option_duration(std::size_t spelling, double v, double a)
{
    if(spelling == 0u)
        return (a + v * v) / (v * a);
    if(spelling == 1u)
        return 1.0 / v + v / a;
    if(spelling == 2u)
        return v / a + 1.0 / v;

    return (a / v + v) / a;
}

// a = v^2/(vT - 1) and its rearrangements.
inline constexpr std::size_t acceleration_spelling_count = 3u;

inline double option_acceleration(std::size_t spelling, double v, double T)
{
    if(spelling == 0u)
        return v * v / (v * T - 1.0);
    if(spelling == 1u)
        return v / (T - 1.0 / v);

    return v * v / (T * v - 1.0);
}

// v = (aT - sqrt(a) sqrt(aT^2 - 4))/2 and its rearrangements.
inline constexpr std::size_t rate_spelling_count = 4u;

inline double option_rate(std::size_t spelling, double a, double T)
{
    if(spelling == 0u)
        return 0.5 * (a * T - std::sqrt(a) * std::sqrt(a * T * T - 4.0));
    if(spelling == 1u)
        return 0.5 * (a * T - std::sqrt(a * a * T * T - 4.0 * a));
    if(spelling == 2u)
        return 2.0 * a / (a * T + std::sqrt(a) * std::sqrt(a * T * T - 4.0));

    return 2.0 / (T + std::sqrt(T * T - 4.0 / a));
}

// Lynch & Park, Modern Robotics, eq. (9.16)-(9.24).
inline trajectory::scaling_sample option_phases(const option_shape &shape, double t, double T, double v, double a)
{
    const bool ramping = shape.ramp == ramp_test::divided ? t <= v / a : a * t <= v;
    if(ramping)
        return {0.5 * a * t * t, a * t, a};
    if(t <= T - v / a)
        return {v * t - v * v / (2.0 * a), v, 0.0};
    if(shape.late == deceleration::printed)
        return {(2.0 * a * v * T - 2.0 * v * v - a * a * (t - T) * (t - T)) / (2.0 * a), a * (T - t), -a};

    return {1.0 - 0.5 * a * (T - t) * (T - t), a * (T - t), -a};
}

inline expected<trajectory::scaling_sample, refusal> option_sampled(const option_shape &shape, double t, double T, double v, double a)
{
    if(const std::optional<trajectory::scaling_sample> rest = resting(shape.ends, t, T))
        return *rest;

    return option_phases(shape, ended(shape.ends, t, T), T, v, a);
}

template<std::size_t Form>
expected<trajectory::scaling_sample, refusal> book_velocity_acceleration_option(double t, double velocity, double acceleration)
{
    constexpr option_shape shape = option_shape_at(Form % option_shape_count);
    if(shape.check == domain_check::printed && !(velocity * velocity / acceleration <= 1.0))
        return unexpected(refusal::unsupported_input);

    return option_sampled(shape, t, option_duration(Form / option_shape_count, velocity, acceleration), velocity, acceleration);
}

template<std::size_t Form>
expected<trajectory::scaling_sample, refusal> book_velocity_duration_option(double t, double duration, double velocity)
{
    constexpr option_shape shape = option_shape_at(Form % option_shape_count);
    if(shape.check == domain_check::printed && !(velocity * duration > 1.0 && velocity * duration <= 2.0))
        return unexpected(refusal::unsupported_input);

    return option_sampled(shape, t, duration, velocity, option_acceleration(Form / option_shape_count, velocity, duration));
}

template<std::size_t Form>
expected<trajectory::scaling_sample, refusal> book_acceleration_duration_option(double t, double duration, double acceleration)
{
    constexpr option_shape shape = option_shape_at(Form % option_shape_count);
    if(shape.check == domain_check::printed && !(acceleration * duration * duration >= 4.0))
        return unexpected(refusal::unsupported_input);

    return option_sampled(shape, t, duration, option_rate(Form / option_shape_count, acceleration, duration), acceleration);
}

using option_slot = expected<trajectory::scaling_sample, refusal> (*)(double t, double first, double second);

inline constexpr std::size_t velocity_acceleration_option_count = duration_spelling_count * option_shape_count;
inline constexpr std::size_t velocity_duration_option_count     = acceleration_spelling_count * option_shape_count;
inline constexpr std::size_t acceleration_duration_option_count = rate_spelling_count * option_shape_count;

template<std::size_t... Index>
std::array<option_slot, sizeof...(Index)> book_velocity_acceleration_options(std::index_sequence<Index...>)
{
    return {&book_velocity_acceleration_option<Index>...};
}

template<std::size_t... Index>
std::array<option_slot, sizeof...(Index)> book_velocity_duration_options(std::index_sequence<Index...>)
{
    return {&book_velocity_duration_option<Index>...};
}

template<std::size_t... Index>
std::array<option_slot, sizeof...(Index)> book_acceleration_duration_options(std::index_sequence<Index...>)
{
    return {&book_acceleration_duration_option<Index>...};
}

inline std::array<option_slot, velocity_acceleration_option_count> book_velocity_acceleration_options()
{
    return book_velocity_acceleration_options(std::make_index_sequence<velocity_acceleration_option_count>{});
}

inline std::array<option_slot, velocity_duration_option_count> book_velocity_duration_options()
{
    return book_velocity_duration_options(std::make_index_sequence<velocity_duration_option_count>{});
}

inline std::array<option_slot, acceleration_duration_option_count> book_acceleration_duration_options()
{
    return book_acceleration_duration_options(std::make_index_sequence<acceleration_duration_option_count>{});
}

}

#endif
