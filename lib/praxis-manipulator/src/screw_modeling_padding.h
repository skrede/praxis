#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_PADDING_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_PADDING_H

#include "praxis/manipulator/screw_modeling_window.h"

#include <vector>
#include <cstddef>

namespace praxis::manipulator {

// A screw whose angular part names no axis translates, so its numbers are an angular and a linear
// part; anything else has an axis, a point on it and a pitch. The boundary is the one the drawing
// reads the same screw at, so a row and the line it is drawn as never disagree about which it is.
screw_modeling_window::parameterization typed_as(const screw_axis &screw);

// The screws a composition supplied, taken back out of the table the window keeps as long as the
// derived chain: entries past `supplied` were padded here and are dropped, and a count above the
// table's own length is reached with explicitly zeroed entries, which name no screw and are there
// only to be counted.
std::vector<screw_axis> as_supplied(const std::vector<screw_axis> &table, std::size_t supplied);

}

#endif
