#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_POLYNOMIAL_SCALINGS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_POLYNOMIAL_SCALINGS_H

#include "book_time_scalings.h"

#include "praxis/trajectory/time_scaling.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <optional>

namespace praxis::tests {

// Cubic and quintic time scalings as Lynch & Park, Modern Robotics, sec. 9.2.2.1 prints them, each
// written in one of the arrangements a reader of that section arrives at.

enum class polynomial_arrangement : std::uint8_t
{
    raw_coefficients,
    written_out,
    normalized
};

struct polynomial_form
{
    polynomial_arrangement arrangement;
    end_handling ends;
};

inline constexpr std::size_t polynomial_form_count = 3u * 3u;

constexpr std::array<polynomial_form, polynomial_form_count> every_polynomial_form()
{
    std::array<polynomial_form, polynomial_form_count> forms{};
    for(std::size_t index = 0; index < forms.size(); ++index)
        forms[index] = polynomial_form{static_cast<polynomial_arrangement>(index / 3u), static_cast<end_handling>(index % 3u)};

    return forms;
}

inline constexpr std::array<polynomial_form, polynomial_form_count> polynomial_forms = every_polynomial_form();

// Eq. (9.9) and (9.10), with a2 = 3/T^2 and a3 = -2/T^3.
inline trajectory::scaling_sample book_cubic_at(polynomial_arrangement arrangement, double t, double T)
{
    if(arrangement == polynomial_arrangement::raw_coefficients)
    {
        const double a2 = 3.0 / (T * T);
        const double a3 = -2.0 / (T * T * T);

        return {a2 * t * t + a3 * t * t * t, 2.0 * a2 * t + 3.0 * a3 * t * t, 2.0 * a2 + 6.0 * a3 * t};
    }
    if(arrangement == polynomial_arrangement::written_out)
        return {3.0 * t * t / (T * T) - 2.0 * t * t * t / (T * T * T), 6.0 * t / (T * T) - 6.0 * t * t / (T * T * T), 6.0 / (T * T) - 12.0 * t / (T * T * T)};

    const double tau = t / T;

    return {3.0 * tau * tau - 2.0 * tau * tau * tau, (6.0 * tau - 6.0 * tau * tau) / T, (6.0 - 12.0 * tau) / (T * T)};
}

// Fig. 9.4: s = a3 t^3 + a4 t^4 + a5 t^5 with a3 = 10/T^3, a4 = -15/T^4 and a5 = 6/T^5.
inline trajectory::scaling_sample book_quintic_at(polynomial_arrangement arrangement, double t, double T)
{
    if(arrangement == polynomial_arrangement::raw_coefficients)
    {
        const double a3 = 10.0 / (T * T * T);
        const double a4 = -15.0 / (T * T * T * T);
        const double a5 = 6.0 / (T * T * T * T * T);

        return {a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t, 3.0 * a3 * t * t + 4.0 * a4 * t * t * t + 5.0 * a5 * t * t * t * t,
                6.0 * a3 * t + 12.0 * a4 * t * t + 20.0 * a5 * t * t * t};
    }
    if(arrangement == polynomial_arrangement::written_out)
        return {10.0 * std::pow(t / T, 3) - 15.0 * std::pow(t / T, 4) + 6.0 * std::pow(t / T, 5),
                (30.0 * std::pow(t / T, 2) - 60.0 * std::pow(t / T, 3) + 30.0 * std::pow(t / T, 4)) / T,
                (60.0 * (t / T) - 180.0 * std::pow(t / T, 2) + 120.0 * std::pow(t / T, 3)) / (T * T)};

    const double tau = t / T;

    return {10.0 * tau * tau * tau - 15.0 * tau * tau * tau * tau + 6.0 * tau * tau * tau * tau * tau, (30.0 * tau * tau - 60.0 * tau * tau * tau + 30.0 * tau * tau * tau * tau) / T,
            (60.0 * tau - 180.0 * tau * tau + 120.0 * tau * tau * tau) / (T * T)};
}

template<std::size_t Form>
expected<trajectory::scaling_sample, refusal> book_cubic(double t, double duration)
{
    constexpr polynomial_form form = polynomial_forms[Form];
    if(const std::optional<trajectory::scaling_sample> rest = resting(form.ends, t, duration))
        return *rest;

    return book_cubic_at(form.arrangement, ended(form.ends, t, duration), duration);
}

template<std::size_t Form>
expected<trajectory::scaling_sample, refusal> book_quintic(double t, double duration)
{
    constexpr polynomial_form form = polynomial_forms[Form];
    if(const std::optional<trajectory::scaling_sample> rest = resting(form.ends, t, duration))
        return *rest;

    return book_quintic_at(form.arrangement, ended(form.ends, t, duration), duration);
}

using polynomial_slot = expected<trajectory::scaling_sample, refusal> (*)(double t, double duration);

template<std::size_t... Index>
std::array<polynomial_slot, sizeof...(Index)> book_cubics(std::index_sequence<Index...>)
{
    return {&book_cubic<Index>...};
}

template<std::size_t... Index>
std::array<polynomial_slot, sizeof...(Index)> book_quintics(std::index_sequence<Index...>)
{
    return {&book_quintic<Index>...};
}

inline std::array<polynomial_slot, polynomial_form_count> book_cubics()
{
    return book_cubics(std::make_index_sequence<polynomial_form_count>{});
}

inline std::array<polynomial_slot, polynomial_form_count> book_quintics()
{
    return book_quintics(std::make_index_sequence<polynomial_form_count>{});
}

}

#endif
