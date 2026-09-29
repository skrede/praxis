#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_PLACEMENT_READING_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_PLACEMENT_READING_H

#include "praxis/manipulator/model_placement.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/axis_order.h"

#include <Eigen/Core>

namespace praxis::manipulator {

// A slot is doubted where it holds its default, or where it answered a value that is not finite
// although it was handed finite arguments.
void note_reading(const rigid_motion::frame_ops &frames, rigid_motion::frame_slot slot, bool handed_finite, bool answered_finite, placement_doubts &doubts);

rotation read_rotation(const rigid_motion::frame_ops &frames, const Eigen::Vector3f &euler_degrees, axis_order order, placement_doubts &doubts);

}

#endif
