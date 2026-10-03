#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_FORMS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_FORMS_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace praxis::tests {

// Runs of via points as Lynch & Park, Modern Robotics, sec. 9.3, eq. (9.26)-(9.29) join them, with
// interior velocities by Biagiotti & Melchiorri, Trajectory Planning for Automatic Machines and
// Robots, sec. 2.1.4, eq. (2.3), each written in one of the arrangements a reader of those sections
// arrives at. Every form gives each segment the time its slowest joint needs at its bound and then
// stretches the run once by the largest ratio of a joint's peak speed to its bound. Fewer than three
// rows go to the reference's straight line.

enum class segment_lookup : std::uint8_t
{
    upper_bound,
    linear_scan
};

enum class peak_root : std::uint8_t
{
    printed,
    derivative
};

enum class stretch_fit : std::uint8_t
{
    refit,
    scaled
};

enum class via_ends : std::uint8_t
{
    held,
    at_rest
};

enum class via_sign : std::uint8_t
{
    three_valued,
    product,
    sign_bit,
    two_valued
};

struct via_form
{
    segment_lookup lookup;
    peak_root peak;
    stretch_fit stretch;
    via_ends ends;
    via_sign sign;
};

inline constexpr std::size_t via_form_count = 2u * 2u * 2u * 2u;

constexpr via_form via_form_at(std::size_t index)
{
    return via_form{static_cast<segment_lookup>(index / 8u % 2u), static_cast<peak_root>(index / 4u % 2u), static_cast<stretch_fit>(index / 2u % 2u), static_cast<via_ends>(index % 2u),
                    via_sign::three_valued};
}

constexpr std::array<via_form, via_form_count> every_via_form()
{
    std::array<via_form, via_form_count> forms{};
    for(std::size_t index = 0; index < forms.size(); ++index)
        forms[index] = via_form_at(index);

    return forms;
}

inline constexpr std::array<via_form, via_form_count> via_forms = every_via_form();

inline constexpr std::array<via_form, 3> via_sign_controls{
        via_form{segment_lookup::upper_bound, peak_root::printed, stretch_fit::refit, via_ends::held, via_sign::product},
        via_form{segment_lookup::upper_bound, peak_root::printed, stretch_fit::refit, via_ends::held, via_sign::sign_bit},
        via_form{segment_lookup::upper_bound, peak_root::printed, stretch_fit::refit, via_ends::held, via_sign::two_valued},
};

inline bool signs_differ(via_sign sign, double before, double after)
{
    if(sign == via_sign::product)
        return before * after < 0.0;
    if(sign == via_sign::sign_bit)
        return std::signbit(before) != std::signbit(after);
    if(sign == via_sign::two_valued)
        return (before > 0.0 ? 1 : -1) != (after > 0.0 ? 1 : -1);

    return (before > 0.0) - (before < 0.0) != (after > 0.0) - (after < 0.0);
}

}

#endif
