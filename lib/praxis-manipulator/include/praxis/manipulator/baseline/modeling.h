#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_MODELING_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_MODELING_H

#include "praxis/manipulator/modeling.h"

// The declaration below matches a slot of modeling_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::manipulator {

expected<screw_chain, refusal> build_chain(const meios::model<> &model);

}

#endif
