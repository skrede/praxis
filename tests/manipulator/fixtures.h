#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_FIXTURES_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_FIXTURES_H

#include "praxis/manipulator.h"
#include "praxis/manipulator/scene_robot.h"

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/rigid_motion/capabilities.h"

#include <threepp/objects/Robot.hpp>
#include <threepp/core/Object3D.hpp>

#include <span>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cstdint>
#include <optional>
#include <filesystem>
#include <system_error>

namespace praxis::fixture {

using namespace manipulator;

inline constexpr float link_length = 0.6f;

// The mirror holds joint values in single precision, so a configuration read back off the rendered
// node returns with the rounding of one float round trip.
inline constexpr double round_trip = 1.0e-6;

inline threepp::Robot::JointInfo revolute(std::string name, std::string parent, std::string child)
{
    return threepp::Robot::JointInfo{threepp::Vector3(0.f, 0.f, 1.f), threepp::Robot::JointType::Revolute, std::move(name), std::nullopt, std::move(parent), std::move(child)};
}

// A renderer robot needs no graphics context: two revolute joints about the world z, the second a
// link length along x from the first, assembled the way the scene builder assembles a parsed model.
inline std::shared_ptr<threepp::Robot> two_joint_handle()
{
    auto handle = std::make_shared<threepp::Robot>();
    for(const char *name : {"base", "upper", "tool"})
    {
        auto link  = threepp::Object3D::create();
        link->name = name;
        handle->addLink(link);
    }

    handle->addJoint(threepp::Object3D::create(), revolute("shoulder", "base", "upper"));

    auto elbow = threepp::Object3D::create();
    elbow->position.set(link_length, 0.f, 0.f);
    handle->addJoint(elbow, revolute("elbow", "upper", "tool"));

    handle->finalize();

    return handle;
}

// Forward kinematics is a pure translation along x by the first joint value, so a pose reported
// through the adapter can be traced back to the joint values the renderer holds.
inline expected<transform, refusal> sliding_forward_kinematics(const transform &, std::span<const screw_axis>, const joint_vector &theta)
{
    transform pose = transform::Identity();
    pose(0, 3)     = theta.size() > 0 ? theta[0] : 0.0;

    return pose;
}

inline screw_chain sliding_chain()
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 1.0);
    bounds.acceleration   = joint_vector::Constant(2, 4.0);
    bounds.lower_position = joint_vector::Constant(2, -1.5);
    bounds.upper_position = joint_vector::Constant(2, 1.5);

    return screw_chain(transform::Identity(), {screw_axis::Zero(), screw_axis::Zero()}, bounds);
}

inline kinematics sliding_solver()
{
    return kinematics::compose(sliding_chain(), forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}, differential_kinematics_ops{}, inverse_kinematics_ops{},
                               rigid_motion::baseline().screw, rigid_motion::baseline().frame)
            .value();
}

// The two-joint chain against the count the two-joint handle reports, which is what lets the holder
// be composed at all: it refuses a chain whose joint count is not the one the renderer reports.
inline scene_robot two_joint_arm(const robot_ops &injected)
{
    return scene_robot::compose(sliding_solver(), injected, rigid_motion::baseline().frame, static_cast<std::uint32_t>(two_joint_handle()->numDOF())).value();
}

inline joint_vector configuration(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

// Concrete implementations in the forms the textbook gives them, so that a test exercising a
// composition drives real mathematics rather than the module's inert defaults.
inline expected<joint_vector, refusal> straight_line(const joint_vector &start, const joint_vector &end, double s)
{
    return joint_vector(start + s * (end - start));
}

inline expected<transform, refusal> interpolated(const transform &start, const transform &end, double s)
{
    return transform(start + s * (end - start));
}

inline expected<trajectory::scaling_sample, refusal> quintic(double t, double duration)
{
    const double x = duration > 0.0 ? t / duration : 0.0;

    return trajectory::scaling_sample{10.0 * std::pow(x, 3) - 15.0 * std::pow(x, 4) + 6.0 * std::pow(x, 5),
                                      (30.0 * std::pow(x, 2) - 60.0 * std::pow(x, 3) + 30.0 * std::pow(x, 4)) / duration,
                                      (60.0 * x - 180.0 * std::pow(x, 2) + 120.0 * std::pow(x, 3)) / (duration * duration)};
}

inline expected<joint_vector, refusal> position_as_configuration(const kinematics &, const transform &pose, const joint_vector &)
{
    return configuration(pose(0, 3), pose(1, 3));
}

inline motion_ops composing_motion()
{
    return motion_ops{.task_space_pose = &position_as_configuration};
}

inline trajectory::path_ops composing_path()
{
    return trajectory::path_ops{.joint_straight_line = &straight_line, .decoupled = &interpolated};
}

inline trajectory::time_scaling_ops composing_time_scaling()
{
    return trajectory::time_scaling_ops{.quintic = &quintic};
}

// A folder fixture lives under the system temporary directory only: one left in the repository or
// the build tree is picked up by the convention gate on the next configure, as an unrelated
// failure. The name separates one suite's tree from another's, which run as concurrent processes.
class scratch_tree
{
public:
    explicit scratch_tree(const std::string &suite)
            : m_root(std::filesystem::temp_directory_path() / ("praxis_" + suite + "_fixture"))
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
        std::filesystem::create_directories(m_root);

        // The library answers resolved paths, and a temporary directory is not always its own
        // resolved form: macOS reaches it through a symlink and Windows through an abbreviated
        // component. A fixture holding the raw path would compare two spellings of one place.
        m_root = std::filesystem::weakly_canonical(m_root);
    }

    scratch_tree(const scratch_tree &)            = delete;
    scratch_tree &operator=(const scratch_tree &) = delete;

    ~scratch_tree()
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
    }

    const std::filesystem::path &root() const
    {
        return m_root;
    }

private:
    std::filesystem::path m_root;
};

}

#endif
