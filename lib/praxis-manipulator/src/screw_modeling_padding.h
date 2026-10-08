#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_PADDING_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_PADDING_H

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/screw_modeling_window.h"

#include <span>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

// A screw whose angular part names no axis translates, so its numbers are an angular and a linear
// part; anything else has an axis, a point on it and a pitch. The boundary is the one the drawing
// reads the same screw at, so a row and the line it is drawn as never disagree about which it is.
screw_modeling_window::parameterization typed_as(const screw_axis &screw);

// One joint's drawable screw: what was supplied for it, or the screw a row nobody supplied opens at.
screw_axis supplied_or_opening(const rigid_motion::screw_ops &turning, const screw_axis &derived, const supplied_screw &held);

// How many screws the table handed to an arm of `rendered` joints holds: every entry it names where
// the derived chain is as long as the arm, and that chain's own count where it is not, so a chain
// the arm cannot take is withheld at that count whatever was supplied.
std::size_t drawn_count(const screw_chain &derived, std::size_t supplied, std::size_t rendered);

// The table handed to the drawing, drawn_count screws long: one per derived joint, padded wherever
// nobody supplied one, followed by the entries named past the last joint, an empty one padded the
// same way. It is composed where it is needed rather than stored, so no screw is written in two
// places.
std::vector<screw_axis> as_drawn(const screw_chain &derived, const rigid_motion::screw_ops &turning, std::span<const supplied_screw> supplied, std::size_t rendered);

}

#endif
