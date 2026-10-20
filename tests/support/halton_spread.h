#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_HALTON_SPREAD_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_HALTON_SPREAD_H

#include "praxis/manipulator/types.h"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <numbers>

namespace praxis::tests {

// The points of the Halton spread an inverse-kinematics window with no start opens at, in order.
constexpr std::array<std::size_t, 8> opening_spread_indices{1, 2, 3, 4, 5, 6, 7, 8};

inline double radical_inverse_of(std::size_t base, std::size_t index)
{
    double inverted = 0.0;
    double fraction = 1.0 / static_cast<double>(base);
    for(std::size_t left = index; left != 0u; left /= base)
    {
        inverted += fraction * static_cast<double>(left % base);
        fraction /= static_cast<double>(base);
    }

    return inverted;
}

// The Halton point at `index` over a whole turn centered on zero, the n-th joint in the n-th prime
// base: Halton, Numerische Mathematik 2 (1960), 84-90. Holds for up to eight joints.
inline manipulator::joint_vector halton_start(std::size_t joints, std::size_t index)
{
    constexpr double turn = 2.0 * std::numbers::pi;
    constexpr std::array<std::size_t, 8> bases{2, 3, 5, 7, 11, 13, 17, 19};

    manipulator::joint_vector start(static_cast<Eigen::Index>(joints));
    for(std::size_t joint = 0; joint < joints; ++joint)
        start[static_cast<Eigen::Index>(joint)] = (radical_inverse_of(bases.at(joint), index) - 0.5) * turn;

    return start;
}

}

#endif
