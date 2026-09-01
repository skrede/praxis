#include "praxis/evaluation/comparators.h"

#include <Eigen/Geometry>

#include <algorithm>

namespace praxis::evaluation {

namespace {

// A pose discrepancy and the discrepancy of a logarithm that names one are the two kinds carrying a
// translational half; every other kind leaves that field at zero.
bool carries_a_linear_half(residual_kind kind)
{
    return kind == residual_kind::pose || kind == residual_kind::log_up_to_branch;
}

}

residual element_wise_residual(const Eigen::Ref<const Eigen::MatrixXd> &first, const Eigen::Ref<const Eigen::MatrixXd> &second)
{
    return residual{residual_kind::element_wise, (first - second).cwiseAbs().maxCoeff(), 0.0};
}

// The angle is read back through Eigen's axis-angle conversion rather than from the trace, which
// loses its precision near zero and near a half turn and can hand the arc cosine an argument
// outside its domain.
residual geodesic_residual(const Eigen::Matrix3d &first, const Eigen::Matrix3d &second)
{
    const Eigen::Matrix3d between(first.transpose() * second);

    return residual{residual_kind::geodesic, Eigen::AngleAxisd(between).angle(), 0.0};
}

residual pose_residual(const Eigen::Matrix4d &first, const Eigen::Matrix4d &second)
{
    const residual turned = geodesic_residual(first.block<3, 3>(0, 0), second.block<3, 3>(0, 0));
    const double moved    = (first.block<3, 1>(0, 3) - second.block<3, 1>(0, 3)).norm();

    return residual{residual_kind::pose, turned.magnitude, moved};
}

residual axis_up_to_sign_residual(const Eigen::Vector<double, 6> &first, const Eigen::Vector<double, 6> &second)
{
    return residual{residual_kind::axis_up_to_sign, std::min((first - second).norm(), (first + second).norm()), 0.0};
}

agreement verdict_of(const residual &seen, const tolerance_pair &allowed)
{
    if(seen.magnitude > allowed.magnitude)
        return agreement::differed;
    if(carries_a_linear_half(seen.kind) && seen.linear_error_metres > allowed.linear_metres)
        return agreement::differed;

    return agreement::agreed;
}

}
