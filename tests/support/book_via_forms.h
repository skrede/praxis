#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_FORMS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_FORMS_H

#include <cmath>
#include <cstdint>

namespace praxis::tests {

// Runs of via points as Lynch & Park, Modern Robotics, sec. 9.3, eq. (9.26)-(9.29) join them, with
// interior velocities by Biagiotti & Melchiorri, Trajectory Planning for Automatic Machines and
// Robots, sec. 2.1.4, eq. (2.3), each written in one of the arrangements a reader of those sections
// arrives at. Every form gives each segment the time its slowest joint needs at its bound and then
// stretches the run once by the largest ratio of a joint's peak speed to its bound. A run of fewer
// than three rows is a rest-to-rest pair.

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

enum class via_sign : std::uint8_t
{
    three_valued,
    product,
    sign_bit,
    two_valued
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
