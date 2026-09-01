#include "praxis/manipulator/compose_arm.h"

#include <span>
#include <memory>
#include <vector>

namespace robotics_course {

namespace {

praxis::expected<praxis::transform, praxis::refusal> forward_kinematics(const praxis::transform &m, std::span<const praxis::screw_axis>, const praxis::manipulator::joint_vector &)
{
    return m;
}

Eigen::Vector3d position_from_pose(const praxis::transform &pose)
{
    return pose.block<3, 1>(0, 3);
}

praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> quintic(double, double)
{
    return praxis::trajectory::scaling_sample{0.0, 0.0, 0.0};
}

praxis::rotation rotate_z(double)
{
    return praxis::rotation::Identity();
}

}

// Each aggregate names the slots this project has written and leaves the rest on their inert
// defaults, which is what a designated initializer over a subset composes.
praxis::manipulator::capabilities arm()
{
    return praxis::manipulator::capabilities{
            .fk    = praxis::manipulator::forward_kinematics_ops{.forward_kinematics = &forward_kinematics},
            .robot = praxis::manipulator::robot_ops{.position_from_pose = &position_from_pose},
    };
}

praxis::trajectory::capabilities shapes()
{
    return praxis::trajectory::capabilities{.time_scaling = praxis::trajectory::time_scaling_ops{.quintic = &quintic}};
}

praxis::rigid_motion::capabilities motions()
{
    return praxis::rigid_motion::capabilities{.frame = praxis::rigid_motion::frame_ops{.rotate_z = &rotate_z}};
}

praxis::manipulator::arm_window_composer no_windows()
{
    return [](const praxis::manipulator::arm_window_inputs &) { return std::vector<std::shared_ptr<praxis::scene::imgui_window>>{}; };
}

// The whole parameter list written by hand against the shipped headers: the bindings above reach the
// preset without anything under lib/ being edited or rebuilt.
std::shared_ptr<praxis::scene::preset> composed_arm(const meios::model<> &description, const praxis::scene::preset_site &site, const praxis::manipulator::joint_vector &initial)
{
    return praxis::manipulator::compose_arm(description, site, praxis::manipulator::attached_models{}, arm(), shapes(), motions(), initial, no_windows());
}

}
