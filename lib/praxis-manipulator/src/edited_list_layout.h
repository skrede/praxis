#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_LIST_LAYOUT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_LIST_LAYOUT_H

#include <imgui.h>

#include <span>
#include <string>
#include <cstddef>

namespace praxis::manipulator {

// What the row standing at that place is called, counted from one so that the first row reads as
// the first rather than as the zeroth.
std::string row_name(std::size_t row);

// The distance from one value of a row to the next, and how wide a run of that many of them is.
// Every kind of row draws its values on this pitch, which is what lets one header stand over the
// columns of any of them. Both are in pixels at the scale the library is drawing at.
float row_value_pitch();
float row_values_width(std::size_t values);

// Where a row's values begin, measured from the left edge of the panel drawing it, for a list of
// that many rows: the two ordering controls and the row's own name stand before them. It is read
// where the cursor stands at the start of a line, and the header is placed at the same offset.
float row_values_offset(std::size_t rows);

// True where the arrow was pressed. A row standing at the end the arrow points toward cannot be
// carried past it, so the control is drawn where the others are and refuses the keyboard with the
// pointer.
bool render_ordering_arrow(const char *id, ImGuiDir toward, bool at_the_end);

// The column names on a line of their own, each standing over the value it names.
void render_row_columns(std::span<const std::string> named, float from);

}

#endif
