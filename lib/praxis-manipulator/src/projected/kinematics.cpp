#include "step_constants.h"

#include "joint_bounds.h"
#include "iterated_solve.h"

#include "praxis/manipulator/projected/kinematics.h"

#include <Eigen/Cholesky>

namespace praxis::manipulator::projected {

namespace {

// W_E over V_b, angular half first.
Eigen::Matrix<double, 6, 1> weights()
{
    Eigen::Matrix<double, 6, 1> diagonal;
    diagonal << rotation_weight, rotation_weight, rotation_weight, translation_weight, translation_weight, translation_weight;

    return diagonal;
}

// T. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the Levenberg-Marquardt Method", IEEE Trans.
// Robotics 27(5), 2011: d = (J_b^T W_E J_b + (E + bias) I)^-1 J_b^T W_E V_b, E = 1/2 V_b^T W_E V_b.
joint_vector sugihara_step(const jacobian &jb, const twist &error)
{
    const Eigen::Matrix<double, 6, 1> diagonal = weights();

    const double energy                = 0.5 * error.dot(diagonal.asDiagonal() * error);
    const Eigen::MatrixXd weighted     = jb.transpose() * diagonal.asDiagonal();
    const Eigen::Index n               = jb.cols();
    const Eigen::MatrixXd coefficients = weighted * jb + (energy + bias) * Eigen::MatrixXd::Identity(n, n);

    return coefficients.llt().solve(weighted * error);
}

// The projected Levenberg-Marquardt method of C. Kanzow, N. Yamashita, M. Fukushima, J. Comput. Appl.
// Math. 172, 2004, local version: theta+ = P(theta + d), with J_b and V_b as Lynch & Park, Modern
// Robotics, sec. 6.2. A projected point equal to theta would repeat to the cap, so it refuses.
expected<joint_vector, refusal> projected_step(const body_target &target, const joint_vector &theta, const jacobian &jb, const twist &error)
{
    const joint_vector next = projected_inside_bounds(target.chain, joint_vector(theta + sugihara_step(jb, error)));
    if(next == theta)
        return unexpected(refusal::no_solution);

    return next;
}

}

// A seed the entry checks would refuse reaches them unprojected, so its refusal is the book solve's.
expected<void, refusal> inverse_kinematics(const rigid_motion::screw_ops &, const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &chain,
                                           const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    const bool projectable = j0.size() == static_cast<Eigen::Index>(chain.joint_count()) && j0.allFinite();

    return iterated(chain, desired, projectable ? projected_inside_bounds(chain, j0) : j0, parameters, &projected_step, answer);
}

}
