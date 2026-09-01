#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_DRAWN_COLUMNS_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_DRAWN_COLUMNS_H

#include "fixtures.h"
#include "drawn_ellipsoids.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>
#include <threepp/core/BufferGeometry.hpp>

#include <threepp/math/Vector3.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <vector>
#include <utility>
#include <cstddef>
#include <algorithm>

namespace praxis::fixture {

// A placement is held in single precision, so a position and a direction read back off a node
// return with the rounding of one float round trip.
inline constexpr double placed_back = 1.0e-6;

// Drawn metres per unit column part, told to the stencil by every case, so that no case depends on
// the value the stencil opens at.
inline constexpr double angular_column_scale = 0.4;
inline constexpr double linear_column_scale  = 0.8;

// The top three rows are the angular part and the bottom three the linear one.
inline Eigen::Matrix<double, 6, 1> twist_of(const Eigen::Vector3d &angular, const Eigen::Vector3d &linear)
{
    Eigen::Matrix<double, 6, 1> made;
    made << angular, linear;

    return made;
}

inline jacobian of_columns(const std::vector<Eigen::Matrix<double, 6, 1>> &twists)
{
    jacobian built(6, static_cast<Eigen::Index>(twists.size()));
    for(std::size_t column = 0; column < twists.size(); ++column)
        built.col(static_cast<Eigen::Index>(column)) = twists[column];

    return built;
}

// Two columns whose angular part and whose linear part are both away from zero and unequal between
// the columns, so an arrow that is placed from the wrong column or the wrong three rows is caught.
inline jacobian two_columns(double scaled)
{
    return of_columns({twist_of(Eigen::Vector3d(0.0, 0.0, scaled), Eigen::Vector3d(0.0, 2.0 * scaled, 0.0)),
                       twist_of(Eigen::Vector3d(scaled, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 3.0 * scaled))});
}

// The same two columns with each column's two parts exchanged, so an arrow taken from the wrong one
// of the two published Jacobians points the wrong way rather than merely standing at another length.
inline jacobian two_columns_exchanged()
{
    return of_columns({twist_of(Eigen::Vector3d(0.0, 2.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)), twist_of(Eigen::Vector3d(0.0, 0.0, 3.0), Eigen::Vector3d(1.0, 0.0, 0.0))});
}

// Only the two Jacobians, the two decompositions and the tool position are read by what is under
// test, but a snapshot has no field a publication may leave out, so the rest is filled with what an
// arm at rest reports.
inline arm_snapshot published_columns(const expected<jacobian, refusal> &space, const expected<jacobian, refusal> &body, const expected<Eigen::Vector3d, refusal> &at,
                                      jacobian_manipulability space_read = refused(), jacobian_manipulability body_read = refused())
{
    arm_snapshot seen   = published(std::move(space_read), std::move(body_read), at);
    seen.space_jacobian = space;
    seen.body_jacobian  = body;

    return seen;
}

inline threepp::Object3D *shaft_of(threepp::Object3D *arrow)
{
    REQUIRE(arrow != nullptr);

    return arrow->getObjectByName<threepp::Object3D>("shaft");
}

// The head keeps its own length at every arrow length, so the shaft carries the whole of the change
// and the head is measured off an arrow standing at a length the case knows rather than named.
inline double shaft_stretch(threepp::Object3D *arrow)
{
    threepp::Object3D *shaft = shaft_of(arrow);
    REQUIRE(shaft != nullptr);

    return static_cast<double>(shaft->scale.y);
}

inline threepp::Object3D *head_of(threepp::Object3D *arrow)
{
    REQUIRE(arrow != nullptr);

    return arrow->getObjectByName<threepp::Object3D>("tip");
}

inline Eigen::Vector3d arrow_at(threepp::Object3D *arrow)
{
    REQUIRE(arrow != nullptr);

    return Eigen::Vector3d(arrow->position.x, arrow->position.y, arrow->position.z);
}

// The arrow is built along the renderer's +Y, so the way it points is that axis carried by the turn
// its placement wrote.
inline Eigen::Vector3d arrow_along(threepp::Object3D *arrow)
{
    REQUIRE(arrow != nullptr);
    threepp::Vector3 out{0.f, 1.f, 0.f};
    out.applyQuaternion(arrow->quaternion);

    return Eigen::Vector3d(out.x, out.y, out.z);
}

// The principal axes are handed in as the identity, so the greatest magnitude on each of a body's
// own coordinate axes is that axis's drawn semi-axis.
inline Eigen::Vector3d body_extents(threepp::Object3D *drawn)
{
    REQUIRE(drawn != nullptr);
    const std::vector<float> &raw = drawn->geometry()->getAttribute<float>("position")->array();

    Eigen::Vector3d worst = Eigen::Vector3d::Zero();
    for(std::size_t at = 0; at + 2 < raw.size(); at += 3)
        for(Eigen::Index axis = 0; axis < 3; ++axis)
            worst[axis] = std::max(worst[axis], std::abs(static_cast<double>(raw[at + static_cast<std::size_t>(axis)])));

    return worst;
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly. The two column scales are told rather than left at the values the stencil opens
// at, so no case depends on either of them.
struct column_stage
{
    column_stage()
            : loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , source(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), source->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
    {
        put(published_columns(two_columns(1.0), two_columns(1.0), Eigen::Vector3d(Eigen::Vector3d::Zero())));
        REQUIRE(shown.initialize().has_value());
        REQUIRE(shown.set_joint_screws(transform::Identity(), two_axes()).has_value());
        REQUIRE(shown.set_pose_path("recorded", two_poses()).has_value());
        shown.set_column_scale(jacobian_block::angular, angular_column_scale);
        shown.set_column_scale(jacobian_block::linear, linear_column_scale);
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

    threepp::Object3D *arrow(std::size_t column, jacobian_block part)
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::jacobian_column_name(column, part));
    }

    threepp::Object3D *body(jacobian_block which)
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::manipulability_ellipsoid_name(which));
    }

    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> source;
    loadable_robot_stencil shown;
};

}

#endif
