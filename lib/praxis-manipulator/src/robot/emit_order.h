#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_EMIT_ORDER_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_EMIT_ORDER_H

#include <meios/model.h>

#include <vector>

namespace praxis::manipulator {

// Link indices in the order they are added to the robot: a pre-order walk from each root that
// visits the shallowest subtree first, so a short branch is emitted before the long one it hangs
// off. Each link appears once however many roots reach it. A model whose topology was never
// reconstructed falls back to declaration order.
std::vector<int> emit_order(const meios::model<> &model);

}

#endif
