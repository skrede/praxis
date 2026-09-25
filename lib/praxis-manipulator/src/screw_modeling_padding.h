#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_PADDING_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_PADDING_H

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/screw_modeling_window.h"

#include <span>
#include <vector>

namespace praxis::manipulator {

// A screw whose angular part names no axis translates, so its numbers are an angular and a linear
// part; anything else has an axis, a point on it and a pitch. The boundary is the one the drawing
// reads the same screw at, so a row and the line it is drawn as never disagree about which it is.
screw_modeling_window::parameterization typed_as(const screw_axis &screw);

// One joint's drawable screw: what was supplied for it, or the screw a row nobody supplied opens at.
screw_axis supplied_or_opening(const rigid_motion::screw_ops &turning, const screw_axis &derived, const supplied_screw &held);

// The table the drawing and the forward map are handed, exactly as long as the derived chain: one
// screw per joint, the padding standing wherever nobody supplied one. It is composed where it is
// needed rather than stored, so no screw is written in two places.
std::vector<screw_axis> as_drawn(const screw_chain &derived, const rigid_motion::screw_ops &turning, std::span<const supplied_screw> supplied);

}

#endif
