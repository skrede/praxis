#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_TABLE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_TABLE_H

#include "praxis/manipulator/screw_chain_difference.h"

#include "praxis/scene/labeled_value_window.h"

#include <cstddef>

namespace praxis::manipulator {

// The lines a chain comparison draws as, appended to the readout the whole-chain comparison already
// answered: the rows it was handed are left exactly where they are, and the home line and one line
// per joint stand after them. Every appended cell is unlabeled, so the run reads as aligned columns;
// a line's first cell states which line it is and the cells after it carry that line's terms, one
// unit to a column -- radians, then metres, then a defect carrying none. A line answering no number
// at all carries one statement in place of its terms. A term with no number and a value that is not
// finite are each stated rather than printed, so no cell ever carries a number the comparison did
// not measure.
scene::readout screw_modeling_reading(const screw_chain_difference &apart, scene::readout carried);

// The appended lines drawn as one table beneath a header row, leaving the rows before
// `whole_chain_rows` to whoever drew them. A readout carrying no line past that point draws nothing.
void render_screw_modeling_table(const scene::readout &shown, std::size_t whole_chain_rows);

}

#endif
