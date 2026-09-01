#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_VELOCITY_KINEMATICS_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_VELOCITY_KINEMATICS_STAGE_H

#include "drawn_columns.h"
#include "window_stage.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <memory>
#include <utility>
#include <cstddef>

namespace praxis::fixture {

// Where the tool stands, away from the space origin, so the anchor a body column is drawn at and the
// anchor a space column is drawn at are two different points.
inline const Eigen::Vector3d stage_tool(0.4, 0.0, 0.0);

// Distinct entries in every cell, and a different set for each of the two Jacobians, so a reading
// that answered the wrong matrix cannot look like one that answered the right one.
inline jacobian six_by(std::size_t joints, double from)
{
    jacobian taken(6, static_cast<Eigen::Index>(joints));
    for(Eigen::Index row = 0; row < taken.rows(); ++row)
        for(Eigen::Index column = 0; column < taken.cols(); ++column)
            taken(row, column) = from + static_cast<double>(row) + 10.0 * static_cast<double>(column);

    return taken;
}

inline arm_snapshot reading_of(const expected<jacobian, refusal> &space, const expected<jacobian, refusal> &body, jacobian_manipulability of_space, jacobian_manipulability of_body)
{
    arm_snapshot seen   = published(std::move(of_space), std::move(of_body), stage_tool);
    seen.space_jacobian = space;
    seen.body_jacobian  = body;

    return seen;
}

// Both matrices carried, and one decomposition of the given singular values behind each of them.
inline arm_snapshot reading_of(const Eigen::Vector3d &values)
{
    return reading_of(six_by(2u, 1.0), six_by(2u, 100.0), decomposed_both(values), decomposed_both(values));
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly. The window reads one publisher and the arm publishes into another, so work the
// arm's strand runs cannot replace the publication a case put under the window. Every drawn length
// is told rather than left at the value the stencil opens at, so no case depends on one.
struct velocity_stage
{
    velocity_stage()
            : loop(scheduler::inline_workers, scheduler::clock_source{&reading})
            , scene(threepp::Scene::create())
            , source(std::make_shared<arm_publisher>())
            , driving(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), source->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
    {
        REQUIRE(shown.initialize().has_value());
        REQUIRE(shown.set_joint_screws(transform::Identity(), two_axes()).has_value());
        REQUIRE(shown.set_jacobian_columns(2u).has_value());
        shown.set_ellipsoid_scale(jacobian_block::angular, angular_scale);
        shown.set_ellipsoid_scale(jacobian_block::linear, linear_scale);
        shown.set_column_scale(jacobian_block::angular, angular_column_scale);
        shown.set_column_scale(jacobian_block::linear, linear_column_scale);
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

    std::shared_ptr<owned_arm> arm()
    {
        const scheduler::strand work = *loop.make_strand();
        const auto driven            = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
        const auto commanded         = std::make_shared<robot_controller>(*driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(),
                                                                          trajectory::trajectory_ops{}, rigid_motion::baseline().screw);
        owned                        = std::make_shared<owned_arm>(work, work, driven, commanded, driving);

        return owned;
    }

    threepp::Object3D *body(jacobian_block which)
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::manipulability_ellipsoid_name(which));
    }

    threepp::Object3D *arrow(std::size_t column, jacobian_block part)
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::jacobian_column_name(column, part));
    }

    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> source;
    std::shared_ptr<arm_publisher> driving;
    std::shared_ptr<owned_arm> owned;
    loadable_robot_stencil shown;
};

}

#endif
