#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_CHAIN_TRANSLATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_CHAIN_TRANSLATION_H

#include "praxis/manipulator/screw_chain.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/screw.h"

#include <cartan/serial/chain/chain_failure.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <span>
#include <vector>
#include <optional>

namespace praxis::manipulator {

using chain_type = cartan::kinematic_chain<double, cartan::dynamic>;

// Absent for a chain the dependency's validated types refuse: one with no joints, a home pose that
// is not a rigid motion, or a screw axis that is not unit-normalized. Joint bounds are the one part
// that degrades rather than fails -- a pair describing no interval leaves that joint free.
std::optional<chain_type> to_cartan_chain(const screw_chain &chain);

// B_i = [Ad_{M^-1}] S_i: Lynch & Park, Modern Robotics, eq. (4.16).
expected<std::vector<screw_axis>, refusal> to_body_screws(const rigid_motion::screw_ops &screw, const transform &m, std::span<const screw_axis> space_screws);

// A length the chain does not have is a cardinality no binding here serves; every other failure the
// dependency names is a value the mathematics cannot use.
refusal refusal_from(cartan::chain_failure failure);

}

#endif
