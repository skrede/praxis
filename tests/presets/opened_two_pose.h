#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_TWO_POSE_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_TWO_POSE_H

#include "drawn_lines.h"
#include "composed_panels.h"
#include "substituted_rigid_motion.h"

#include "praxis/presets/two_pose.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/two_pose_window.h"
#include "praxis/rigid_motion/decoupled_path_window.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/objects/Line.hpp>

#include <threepp/core/Object3D.hpp>
#include <threepp/core/BufferGeometry.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::fixture {

using namespace praxis::rigid_motion;

// Where each control the panel offers stands below its first item. The two poses are drawn in the
// order the settings carry them, each as an axis order, three angles and three positions.
constexpr std::size_t end_angle_controls    = 8;
constexpr std::size_t end_position_controls = 11;
constexpr std::size_t parameter_control     = 14;

// Written out rather than read off the window, which is what makes the check below one on the count
// the sampling measurement settled rather than a comparison of a constant with itself.
constexpr std::size_t settled_path_points = 33;

inline expected<std::pair<screw_axis, double>, refusal> reversed_screw(const transform &tf)
{
    return matrix_logarithm_se3(inverse(tf));
}

inline expected<std::pair<Eigen::Vector3d, double>, refusal> reversed_turn(const rotation &r)
{
    return matrix_logarithm_so3(rotation(r.transpose()));
}

// The objects and the controls the shipped scenario composes, written out here so that a
// composition leaving the second window out can be built beside it.
inline std::vector<stencil_object> five_objects()
{
    const object_body nothing{body_shape::none, 0.0, nullptr};

    return {stencil_object{"Start", axes_settings{}, nothing}, stencil_object{"End", axes_settings{}, nothing},
            stencil_object{"Body", axes_settings{}, object_body{body_shape::cube, 0.30, nullptr}}, stencil_object{"Path", axes_settings{false}, nothing},
            stencil_object{"Decoupled path", axes_settings{false}, nothing}};
}

inline two_pose_window::settings opening()
{
    return two_pose_window::settings{two_pose_window::pose_controls{Eigen::Vector3f{0.6f, -0.5f, 0.3f}, Eigen::Vector3f::Zero(), axis_order::xyz},
                                     two_pose_window::pose_controls{Eigen::Vector3f{-0.6f, 0.5f, 0.9f}, Eigen::Vector3f{0.f, 90.f, 90.f}, axis_order::xyz}, 0.f};
}

inline two_pose_window::path_route path_into(frame_stencil &target, std::size_t object)
{
    return [&target, object](std::span<const transform> travelled)
    {
        if(travelled.empty())
            return target.set_body(object, object_body{body_shape::none, 0.0, nullptr});

        std::vector<threepp::Vector3> points;
        for(const transform &put : travelled)
            points.emplace_back(static_cast<float>(put(0, 3)), static_cast<float>(put(1, 3)), static_cast<float>(put(2, 3)));

        auto drawn = threepp::BufferGeometry::create();
        drawn->setFromPoints(points);

        target.set_body(object, object_body{body_shape::mesh, 0.0, threepp::Line::create(drawn, threepp::LineBasicMaterial::create())});
    };
}

// One stencil and the coupled controls over it, with no second window anywhere.
struct alone
{
    alone()
            : target()
            , body(target, five_objects(), baseline().frame)
            , driving("Two poses", body, baseline(), path_into(body, two_pose_window::path_object), opening())
    {
        REQUIRE(body.initialize().has_value());
        driving.initialize();
    }

    alone(const alone &) = delete;

    std::vector<Eigen::Vector3d> decoupled()
    {
        return fixture::line_points(target, body.name_of(decoupled_path_window::path_object));
    }

    threepp::Scene target;
    frame_stencil body;
    two_pose_window driving;
};

inline frame_stencil &placed_in(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<frame_stencil &>(*composed->stencil);
}

inline two_pose_window &panel_of(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<two_pose_window &>(*composed->windows.front());
}

inline std::shared_ptr<scene::preset> opened(stage &headless, const capabilities &motions)
{
    const std::shared_ptr<scene::preset> composed = presets::two_pose_preset(headless.site(), motions);

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
        panel->initialize();

    return composed;
}

// The decoupled window reads its poses from the coupled one, so a case that moves a control renders
// both panels before reading what either drew.
inline void settled(const std::shared_ptr<scene::preset> &composed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw(
            [&composed]
            {
                for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
                    panel->render();
            });
}

inline std::vector<Eigen::Vector3d> drawn(stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t object)
{
    return fixture::line_points(*headless.scene, placed_in(composed).name_of(object));
}

inline std::vector<Eigen::Vector3d> coupled_path(stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return drawn(headless, composed, two_pose_window::path_object);
}

inline std::vector<Eigen::Vector3d> decoupled_path(stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return drawn(headless, composed, decoupled_path_window::path_object);
}

// The end pose typed onto the start pose, control by control, which is a state a learner reaches by
// typing and no number of steps arrives at.
inline void type_the_end_onto_the_start(const std::shared_ptr<scene::preset> &composed)
{
    const two_pose_window::pose_controls start = panel_of(composed).state().start;
    const char *const angles[3]{"0", "0", "0"};
    const char *const positions[3]{"0.6", "-0.5", "0.3"};

    REQUIRE(start.euler_degrees.isZero());
    for(std::size_t axis = 0; axis < 3; ++axis)
    {
        fixture::type_into_slider_at(panel_of(composed), end_angle_controls + axis, angles[axis]);
        fixture::type_at(panel_of(composed), end_position_controls + axis, positions[axis]);
    }
}

}

#endif
