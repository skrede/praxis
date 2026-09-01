#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_CHAIN_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_CHAIN_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <meios/model.h>

#include <string>
#include <vector>

namespace praxis::manipulator {

bool is_actuated(meios::joint_kind kind);

// Null for a root link, which no joint reaches down to, and for any index the model's own tables
// do not address.
const meios::joint<> *joint_above(const meios::model<> &model, int link);

expected<int, refusal> root_link(const meios::model<> &model);

// The link indices from the model's root down to the tip, root first.
expected<std::vector<int>, refusal> link_chain(const meios::model<> &model, const std::string &tip_link);

}

#endif
