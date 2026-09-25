#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_TABLE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_TABLE_H

#include "praxis/manipulator/screw_chain_difference.h"

#include "praxis/evaluation/residual.h"

#include "praxis/scene/labeled_value_window.h"

namespace praxis::manipulator {

// The whole chain's own two lines: how far the pose the supplied chain reaches stands from the pose
// the derived one reaches, as a rotation in radians and a distance in metres carried apart and never
// summed. Each is one labeled cell, drawn as this window has always drawn it.
scene::readout whole_chain_lines(const evaluation::residual &apart);

// The same two lines where no pose was reached, each carrying `said` in place of its number so that
// the lines appended beneath them -- which reach no pose either way -- are still drawn.
scene::readout whole_chain_without_pose(const char *said);

// The lines a chain comparison draws as, appended to the two whole-chain lines it is handed: the
// rows it was given are left exactly where they are, and the home line, one line per joint and a
// surplus line where there is one stand after them. Every appended cell is unlabeled, so the run
// reads as aligned columns; a line's first cell states which line it is and the cells after it carry
// that line's terms, one unit to a column -- radians, then metres, then a defect carrying none. A
// line answering no number at all carries one statement in place of its terms. A term with no number
// and a value that is not finite are each stated rather than printed, so no cell ever carries a
// number the comparison did not measure.
scene::readout screw_modeling_reading(const screw_chain_difference &apart, scene::readout carried);

// The whole reading drawn: the two whole-chain lines as a label and six decimals each, or as a label
// and what that line states, and the lines beneath them as one table under a header row. A readout
// carrying a message draws that alone.
void render_screw_modeling_reading(const scene::readout &shown);

}

#endif
