#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_POSE_PATH_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_POSE_PATH_H

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Color.hpp>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <string>

namespace praxis::manipulator {

// The tone a path is drawn in when the composition names none.
threepp::Color opening_path_tone();

// The points are read in the frame the object is hung in and joined in the order they arrive. Fewer
// than two points is no drawing at all and answers null rather than an empty line.
std::shared_ptr<threepp::Object3D> pose_path_object(std::string name, std::span<const Eigen::Vector3d> through, threepp::Color tone);

}

#endif
