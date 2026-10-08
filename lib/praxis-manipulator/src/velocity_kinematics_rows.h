#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_VELOCITY_KINEMATICS_ROWS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_VELOCITY_KINEMATICS_ROWS_H

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/labeled_value_window.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <imgui.h>

#include <cstddef>
#include <optional>

namespace praxis::manipulator {

// A part's plain or highlighted tone in the color space the renderer encodes to on output, which is
// the space a panel writes its own colors in.
ImU32 as_written(jacobian_block part, bool highlighted);

// Whether the lengths this block would be drawn at under this reading and this scale are not all
// finite. A block that is a refusal answers false: it carries no lengths to run away.
bool ellipsoid_unbounded(const expected<manipulability_ellipsoid, refusal> &block, ellipsoid_view read, double scale);

// Whether either block of one decomposition is unbounded under this reading and these scales.
bool either_ellipsoid_unbounded(const jacobian_manipulability &both, ellipsoid_view read, double angular_scale, double linear_scale);

// The Jacobian `frame` names, as six unlabeled rows of one cell per joint in the matrix's own order,
// followed by two rows per block: the block's name carrying its three singular values, and beneath
// it that block's manipulability measure and condition number, named without the block. A block that
// is a refusal carries one row saying so in place of both, and a condition number the publication
// does not carry is stated absent rather than printed; no cell ever carries a number that is not
// finite.
//
// A publication that has not arrived, a Jacobian that is a refusal, and a decomposition whose two
// blocks are both refusals each answer a message and no rows, naming which. A block whose drawn
// lengths run away carries a cell saying so at the end of its scalars row. The marked column's top
// three cells are filled in the tone its angular arrow wears and its bottom three in the linear one's,
// and a column the matrix does not have marks nothing. Lynch & Park, Modern Robotics, sections 5.1 and 5.4.
scene::readout velocity_kinematics_reading(const arm_snapshot *seen, jacobian_frame frame, ellipsoid_view read, double angular_scale, double linear_scale,
                                           std::optional<std::size_t> marked);

}

#endif
