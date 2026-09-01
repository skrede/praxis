#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_DRAWN_ELLIPSOIDS_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_DRAWN_ELLIPSOIDS_H

#include "fixtures.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <utility>
#include <cstddef>
#include <optional>

namespace praxis::fixture {

// The vertices are held in single precision, so an extent read back off the buffer is comparable at
// one float round trip and no closer.
inline constexpr double read_back = 1.0e-6;

// Drawn metres per unit semi-axis, and the cut as a multiple of a block's own scale, told to the
// stencil by every case so that no case depends on the value the stencil opens at. A block's cut in
// drawn metres is that multiple times that block's scale: 0.15 for the angular block and 0.3 for the
// linear one.
inline constexpr double angular_scale = 0.25;
inline constexpr double linear_scale  = 0.5;
inline constexpr double cap_ratio     = 0.6;
inline constexpr double angular_cap   = cap_ratio * angular_scale;
inline constexpr double linear_cap    = cap_ratio * linear_scale;

// The principal axes are the identity, so a body's own coordinate axes are its principal ones.
inline manipulability_ellipsoid decomposed(const Eigen::Vector3d &values)
{
    const std::optional<double> condition = values.z() > 0.0 ? std::optional<double>(values.x() / values.z()) : std::nullopt;

    return manipulability_ellipsoid{values, Eigen::Matrix3d::Identity(), values.prod(), condition};
}

inline jacobian_manipulability decomposed_both(const Eigen::Vector3d &values)
{
    return jacobian_manipulability{decomposed(values), decomposed(values)};
}

inline jacobian_manipulability refused()
{
    return jacobian_manipulability{unexpected(refusal::unsupported_input), unexpected(refusal::unsupported_input)};
}

// Only the two decompositions and the tool position are read by what is under test, but a snapshot
// has no field a publication may leave out, so the rest is filled with what an arm at rest reports.
inline arm_snapshot published(jacobian_manipulability space, jacobian_manipulability body, const expected<Eigen::Vector3d, refusal> &at)
{
    const transform put = transform::Identity();
    const rotation upright(rotation::Identity());

    return arm_snapshot{configuration(0.0, 0.0),
                        joint_limits{},
                        put,
                        put,
                        put,
                        at,
                        Eigen::Vector3d(Eigen::Vector3d::Zero()),
                        upright,
                        upright,
                        recording_parameters{},
                        1.0,
                        false,
                        scheduler::task_counters{},
                        {},
                        unexpected(refusal::not_implemented),
                        unexpected(refusal::not_implemented),
                        std::move(space),
                        std::move(body),
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

inline std::vector<screw_axis> two_axes()
{
    const rigid_motion::screw_ops screw = rigid_motion::baseline().screw;

    return {screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0).value(),
            screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d(static_cast<double>(link_length), 0.0, 0.0), Eigen::Vector3d::UnitZ(), 0.0).value()};
}

inline std::vector<transform> two_poses()
{
    transform first  = transform::Identity();
    transform second = transform::Identity();
    first(0, 3)      = 0.25;
    second(1, 3)     = 0.5;

    return {first, second};
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly. The two ellipsoid scales and the cap are told rather than left at the values the
// stencil opens at, so no case depends on any of the three. A stage built unsized tells neither, so a
// case can read what the stencil opens at.
struct ellipsoid_stage
{
    explicit ellipsoid_stage(bool sized = true)
            : loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , source(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), source->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
    {
        put(published(refused(), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
        REQUIRE(shown.initialize().has_value());
        REQUIRE(shown.set_joint_screws(transform::Identity(), two_axes()).has_value());
        REQUIRE(shown.set_pose_path("recorded", two_poses()).has_value());
        if(!sized)
            return;
        shown.set_ellipsoid_scale(jacobian_block::angular, angular_scale);
        shown.set_ellipsoid_scale(jacobian_block::linear, linear_scale);
        shown.set_force_cap_ratio(cap_ratio);
    }

    void put(const arm_snapshot &seen)
    {
        source->publish(std::make_shared<const arm_snapshot>(seen));
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    threepp::Object3D *body(jacobian_block which)
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::manipulability_ellipsoid_name(which));
    }

    threepp::Object3D *line(jacobian_block which, std::size_t axis, bool forward)
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::ellipsoid_continuation_name(which, axis, forward));
    }

    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> source;
    loadable_robot_stencil shown;
};

}

#endif
