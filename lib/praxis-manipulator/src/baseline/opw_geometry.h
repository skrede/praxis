#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_OPW_GEOMETRY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_OPW_GEOMETRY_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <cartan/analytical/analytical_types.h>
#include <cartan/analytical/solver_opw.h>

namespace praxis::manipulator {

// The seven lengths, the six offsets and the six sign corrections of the ortho-parallel basis with a
// spherical wrist, read off the chain's own axis lines and home pose: Brandstotter, Angerer &
// Hofbaur (2014). Refused for a chain of other than six revolute joints, for a shoulder axis that is
// not orthogonal to the base axis, for an elbow axis that is not parallel to the shoulder axis, and
// for last three axes meeting at no common point -- each names a kind of arm this decomposition does
// not serve rather than a chain that is ill formed.
expected<cartan::opw_parameters<double>, refusal> to_opw_parameters(const screw_chain &chain);

// What the derived parameters are trusted on: the forward map they carry and the chain's own forward
// map are asked for the same configurations, and a disagreement leaves the parameters describing
// some arm other than the one the chain describes.
expected<void, refusal> agrees_with_chain(const forward_kinematics_ops &forward, const screw_chain &chain, const cartan::opw_parameters<double> &parameters);

// A target outside the workspace, a decomposition that placed no candidate and a candidate no
// reconstruction accepted are all answers about the request; a geometry the closed form does not
// serve is a kind it does not serve, and a nonfinite value is one the mathematics cannot use.
refusal refusal_from(cartan::analytical_failure failure);

}

#endif
