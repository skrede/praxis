#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_TIME_SCALINGS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_TIME_SCALINGS_H

#include "praxis/trajectory/time_scaling.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

namespace praxis::tests {

// Trapezoidal time scalings as Lynch & Park, Modern Robotics, sec. 9.2.2.2 prints them, each written
// in one of the arrangements a reader of that section arrives at. Every form solves its cruise rate
// from the duration it is handed by the section's third option, with no guard on the radicand.

enum class apex_test : std::uint8_t
{
    rate_squared_over_change,
    rate_above_root
};

enum class coast_duration : std::uint8_t
{
    printed,
    split
};

enum class apex_duration : std::uint8_t
{
    inverse_root,
    root_over_change,
    halves
};

enum class rate_solve : std::uint8_t
{
    printed,
    single_root,
    rationalized
};

enum class duration_check : std::uint8_t
{
    natural,
    solved_rate,
    none
};

enum class deceleration : std::uint8_t
{
    printed,
    from_the_end
};

enum class end_handling : std::uint8_t
{
    as_written,
    held
};

struct trapezoid_form
{
    apex_test apex_when;
    coast_duration coast;
    apex_duration apex;
    rate_solve solve;
    duration_check check;
    deceleration late;
    end_handling ends;
};

inline constexpr std::size_t trapezoid_form_count = 2u * 2u * 3u * 3u * 3u * 2u * 2u;

constexpr trapezoid_form trapezoid_form_at(std::size_t index)
{
    trapezoid_form form{};
    form.ends      = static_cast<end_handling>(index % 2u);
    form.late      = static_cast<deceleration>(index / 2u % 2u);
    form.check     = static_cast<duration_check>(index / 4u % 3u);
    form.solve     = static_cast<rate_solve>(index / 12u % 3u);
    form.apex      = static_cast<apex_duration>(index / 36u % 3u);
    form.coast     = static_cast<coast_duration>(index / 108u % 2u);
    form.apex_when = static_cast<apex_test>(index / 216u % 2u);

    return form;
}

constexpr std::array<trapezoid_form, trapezoid_form_count> every_trapezoid_form()
{
    std::array<trapezoid_form, trapezoid_form_count> forms{};
    for(std::size_t index = 0; index < forms.size(); ++index)
        forms[index] = trapezoid_form_at(index);

    return forms;
}

inline constexpr std::array<trapezoid_form, trapezoid_form_count> trapezoid_forms = every_trapezoid_form();

inline double book_natural_duration(const trapezoid_form &form, double v, double a)
{
    const bool apex = form.apex_when == apex_test::rate_squared_over_change ? v * v / a > 1.0 : v > std::sqrt(a);
    if(!apex)
        return form.coast == coast_duration::printed ? (a + v * v) / (v * a) : 1.0 / v + v / a;
    if(form.apex == apex_duration::inverse_root)
        return 2.0 / std::sqrt(a);
    if(form.apex == apex_duration::root_over_change)
        return 2.0 * std::sqrt(a) / a;

    return std::sqrt(a) / a + std::sqrt(a) / a;
}

inline double book_cruise_rate(rate_solve solve, double T, double a)
{
    if(solve == rate_solve::printed)
        return 0.5 * (a * T - std::sqrt(a) * std::sqrt(a * T * T - 4.0));
    if(solve == rate_solve::single_root)
        return 0.5 * (a * T - std::sqrt(a * a * T * T - 4.0 * a));

    return 2.0 / (T + std::sqrt(T * T - 4.0 / a));
}

// Lynch & Park, Modern Robotics, eq. (9.16)-(9.24).
inline trajectory::scaling_sample book_phases(deceleration late, double t, double T, double v, double a)
{
    if(t <= v / a)
        return {0.5 * a * t * t, a * t, a};
    if(t <= T - v / a)
        return {v * t - v * v / (2.0 * a), v, 0.0};
    if(late == deceleration::printed)
        return {(2.0 * a * v * T - 2.0 * v * v - a * a * (t - T) * (t - T)) / (2.0 * a), a * (T - t), -a};

    return {1.0 - 0.5 * a * (T - t) * (T - t), a * (T - t), -a};
}

template<std::size_t Form>
expected<trajectory::scaling_sample, refusal> book_trapezoid(double t, double duration, double max_velocity, double max_acceleration)
{
    constexpr trapezoid_form form = trapezoid_forms[Form];
    if(form.check == duration_check::natural && duration < book_natural_duration(form, max_velocity, max_acceleration))
        return unexpected(refusal::unsupported_input);

    const double rate = book_cruise_rate(form.solve, duration, max_acceleration);
    if(form.check == duration_check::solved_rate && rate > max_velocity)
        return unexpected(refusal::unsupported_input);

    const double at = form.ends == end_handling::held ? std::clamp(t, 0.0, duration) : t;

    return book_phases(form.late, at, duration, rate, max_acceleration);
}

using trapezoid_slot = expected<trajectory::scaling_sample, refusal> (*)(double t, double duration, double max_velocity, double max_acceleration);

template<std::size_t... Index>
std::array<trapezoid_slot, sizeof...(Index)> book_trapezoids(std::index_sequence<Index...>)
{
    return {&book_trapezoid<Index>...};
}

inline std::array<trapezoid_slot, trapezoid_form_count> book_trapezoids()
{
    return book_trapezoids(std::make_index_sequence<trapezoid_form_count>{});
}

}

#endif
