#include "window_stage.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <memory>
#include <vector>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

const rigid_motion::frame_ops reference_frame = rigid_motion::baseline().frame;

// The world object's assignment builds under this order and takes no selector; the tool's two are
// given it by the settings below.
constexpr axis_order transform_order = axis_order::zyx;

// No two components of a triple equal, and none of them zero, a right angle or a straight one, so
// all three faults land clear of the correct value. No component is shared between the two tool
// triples either, so a case reading the wrong recorded row fails rather than passing by coincidence.
const Eigen::Vector3f world_object_degrees{41.f, 58.f, -23.f};
const Eigen::Vector3f tool_graphics_degrees{31.f, 47.f, -13.f};
const Eigen::Vector3f tool_kinematics_degrees{29.f, -61.f, 17.f};

// The windows hold their angles in single precision, so a value reaching the seam is the degrees
// behind it to within one float step, which is 6.7e-8 radians at the widest of these angles.
constexpr double conversion_tolerance = 1.0e-7;

std::vector<Eigen::Vector3d> built_radians;

rotation recorded_build(const Eigen::Vector3d &euler_radians, axis_order order)
{
    built_radians.push_back(euler_radians);

    return reference_frame.rotation_matrix_from_euler(euler_radians, order);
}

rigid_motion::frame_ops framing()
{
    rigid_motion::frame_ops ops    = reference_frame;
    ops.rotation_matrix_from_euler = &recorded_build;

    return ops;
}

Eigen::Vector3d radians_of(const Eigen::Vector3f &degrees)
{
    return degrees.cast<double>() * radians_per_degree;
}

// Against the constant written out, never through the helper the site under test calls: an
// assertion routed through that helper cancels every perturbation of its use.
void reaches(const Eigen::Vector3d &recorded_radians, const Eigen::Vector3f &degrees)
{
    const Eigen::Vector3d wanted = radians_of(degrees);
    for(Eigen::Index angle = 0; angle < 3; ++angle)
        CHECK(recorded_radians[angle] == Catch::Approx(wanted[angle]).margin(conversion_tolerance));
}

void agrees(const rotation &seated, const rotation &built)
{
    for(Eigen::Index row = 0; row < 3; ++row)
        for(Eigen::Index column = 0; column < 3; ++column)
            CHECK(seated(row, column) == Catch::Approx(built(row, column)).margin(conversion_tolerance));
}

struct staged
{
    composed_arm arm;
    std::shared_ptr<threepp::Scene> target;
    std::shared_ptr<loadable_robot_stencil> shown;
};

staged stage_with(praxis::scheduler::scheduler &loop, attached_models attached)
{
    composed_arm arm                             = compose(loop, composing_motion(), rigid_motion::baseline().screw);
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    auto shown = std::make_shared<loadable_robot_stencil>(two_joint_handle(), std::move(attached), *target, loop.main_strand(), arm.seen, praxis::rigid_motion::baseline().screw,
                                                          praxis::rigid_motion::screw_slot_set{});

    return staged{std::move(arm), target, std::move(shown)};
}

world_object_window::settings placing()
{
    return world_object_window::settings{
            true, "", world_object_window::world_view::transform, Eigen::Vector3f::Ones(), Eigen::Vector3f::Zero(), world_object_degrees,
    };
}

tool_window::settings attaching()
{
    const Eigen::Vector3f none = Eigen::Vector3f::Zero();

    return tool_window::settings{
            true, "", tool_window::tool_view::kinematics_transform, tool_graphics_degrees, transform_order, Eigen::Vector3f::Ones(), none, tool_kinematics_degrees, transform_order,
            none,
    };
}

}

TEST_CASE("the orientation a world object is placed with reaches the frame seam as the same orientation in radians", "[manipulator][world]")
{
    built_radians.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    staged placed = stage_with(loop, attached_models{.world = threepp::Object3D::create()});
    world_object_window panel("World object", *placed.shown, framing(), placing(), "machine/world");
    panel.initialize();

    REQUIRE(built_radians.size() == 1u);
    reaches(built_radians.back(), world_object_degrees);

    placed.shown->tear_down();
}

TEST_CASE("the orientation a tool's rendered transform is built from reaches the frame seam as the same orientation in radians", "[manipulator][tool]")
{
    built_radians.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    staged placed = stage_with(loop, attached_models{.tool = threepp::Object3D::create()});
    tool_window panel("Tool settings", *placed.shown, placed.arm.seen, placed.arm.owned, framing(), attaching(), "machine/tool");
    panel.initialize();
    static_cast<void>(loop.drain());

    // Seating runs both assignments through this slot in one call, in the order its own two
    // statements fix: the rendered transform, then the kinematic offset. Nothing in the recorded
    // rows says which is which, so an edit that swapped those statements would swap the two cases.
    REQUIRE(built_radians.size() == 2u);
    reaches(built_radians.front(), tool_graphics_degrees);

    placed.shown->tear_down();
}

TEST_CASE("the orientation a tool's kinematic offset is built from reaches the frame seam as the same orientation in radians", "[manipulator][tool]")
{
    built_radians.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    staged placed = stage_with(loop, attached_models{.tool = threepp::Object3D::create()});
    tool_window panel("Tool settings", *placed.shown, placed.arm.seen, placed.arm.owned, framing(), attaching(), "machine/tool");
    panel.initialize();
    static_cast<void>(loop.drain());

    REQUIRE(built_radians.size() == 2u);
    reaches(built_radians.back(), tool_kinematics_degrees);

    // The offset is posted through the ownership gate, which the interception above does not cross,
    // so the readback fails for a different reason than the recorded row and cannot mask it.
    const std::shared_ptr<const arm_snapshot> seen = placed.arm.seen.read();
    REQUIRE(seen != nullptr);
    agrees(seen->tool_offset.topLeftCorner<3, 3>(), reference_frame.rotation_matrix_from_euler(radians_of(tool_kinematics_degrees), transform_order));

    placed.shown->tear_down();
}
