#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MOTION_DRAWINGS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MOTION_DRAWINGS_H

#include <threepp/math/Color.hpp>

#include <cstddef>

namespace praxis::manipulator {

// Two windows draw the run the tool traversed, so what it is called and what tone it takes stand
// here rather than on either of them.

// The names the two drawings stand under on the stencil. They are orthogonal drawings: clearing one
// reaches neither the other nor the shapes a comparison draws.
inline constexpr const char *commanded_motion_path = "commanded";
inline constexpr const char *traversed_motion_path = "traversed";

// The tone the path the tool actually traversed is drawn in. The commanded path and the compared
// shapes take the tone a drawn path opens at, so only the one that must be told apart from them is
// named.
inline constexpr threepp::Color::ColorName traversed_motion_tone = threepp::Color::deeppink;

// A run of fewer than two poses is no drawing at all, and the stencil declines it by name.
inline constexpr std::size_t least_drawn_poses = 2;

}

#endif
