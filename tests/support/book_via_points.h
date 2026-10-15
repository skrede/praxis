#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_POINTS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_POINTS_H

namespace praxis::tests {

struct book_cubic
{
    double a0, a1, a2, a3;
};

// Lynch & Park, Modern Robotics, eq. (9.26)-(9.29).
inline book_cubic book_segment(double q0, double q1, double v0, double v1, double T)
{
    return {q0, v0, (3.0 * q1 - 3.0 * q0 - 2.0 * v0 * T - v1 * T) / (T * T), (2.0 * q0 + (v0 + v1) * T - 2.0 * q1) / (T * T * T)};
}

}

#endif
