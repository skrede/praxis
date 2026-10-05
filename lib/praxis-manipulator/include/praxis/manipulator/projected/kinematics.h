#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_PROJECTED_KINEMATICS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_PROJECTED_KINEMATICS_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

// The projected Levenberg-Marquardt method of Kanzow, Yamashita & Fukushima (J. Comput. Appl. Math. 172,
// 2004), local version, damped as Sugihara (IEEE Trans. Robotics 27(5), 2011): every iterate lies inside
// the chain's joint bounds.
namespace praxis::manipulator::projected {

expected<void, refusal> inverse_kinematics(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const differential_kinematics_ops &differential,
                                           const screw_chain &chain, const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer);

}

#endif
