#ifndef HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_TWO_POSE_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_TWO_POSE_STAGE_H

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/two_pose_window.h"
#include "praxis/rigid_motion/decoupled_path_window.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <functional>

namespace praxis::fixture {

inline std::vector<rigid_motion::stencil_object> five_objects()
{
    const rigid_motion::object_body nothing{rigid_motion::body_shape::none, 0.0, nullptr};

    return {rigid_motion::stencil_object{"Start", rigid_motion::axes_settings{}, nothing}, rigid_motion::stencil_object{"End", rigid_motion::axes_settings{}, nothing},
            rigid_motion::stencil_object{"Body", rigid_motion::axes_settings{}, rigid_motion::object_body{rigid_motion::body_shape::cube, 0.25, nullptr}},
            rigid_motion::stencil_object{"Path", rigid_motion::axes_settings{false}, nothing},
            rigid_motion::stencil_object{"Decoupled path", rigid_motion::axes_settings{false}, nothing}};
}

inline rigid_motion::two_pose_window::pose_controls placed(const Eigen::Vector3f &position, const Eigen::Vector3f &degrees)
{
    return rigid_motion::two_pose_window::pose_controls{position, degrees, axis_order::xyz};
}

// Both poses carry a turn about all three axes, so an axis order taken in a different order is a
// different rotation and the control that names it moves the pose it belongs to.
inline rigid_motion::two_pose_window::settings opening()
{
    return rigid_motion::two_pose_window::settings{placed({0.6f, -0.5f, 0.3f}, {10.f, 20.f, 30.f}), placed({-0.6f, 0.5f, 0.9f}, {-40.f, 50.f, 150.f}), 0.f};
}

inline rigid_motion::two_pose_window::settings at_parameter(float parameter)
{
    rigid_motion::two_pose_window::settings chosen = opening();
    chosen.parameter                               = parameter;

    return chosen;
}

inline rigid_motion::two_pose_window::settings twice(const rigid_motion::two_pose_window::pose_controls &named)
{
    return rigid_motion::two_pose_window::settings{named, named, 0.f};
}

// Two poses of one orientation, whose screw is a pure slide: the path along it is the straight line
// between the two positions, which is the path the decoupled window draws for any pair.
inline rigid_motion::two_pose_window::settings slide_only()
{
    return rigid_motion::two_pose_window::settings{placed({0.6f, -0.5f, 0.3f}, {10.f, 20.f, 30.f}), placed({-0.6f, 0.5f, 0.9f}, {10.f, 20.f, 30.f}), 0.f};
}

inline rigid_motion::two_pose_window::path_route recording_into(std::size_t &invocations, std::vector<transform> &sampled)
{
    return [&invocations, &sampled](std::span<const transform> travelled)
    {
        ++invocations;
        sampled.assign(travelled.begin(), travelled.end());
    };
}

inline bool alike(const std::vector<transform> &first, const std::vector<transform> &second)
{
    if(first.size() != second.size())
        return false;

    for(std::size_t at = 0; at < first.size(); ++at)
        if(!is_approx_equal(first[at], second[at], 1.0e-9))
            return false;

    return true;
}

// The greatest distance the two paths stand apart at equal parameter, which is what the scenario
// exists to make visible.
inline double apart(const std::vector<transform> &first, const std::vector<transform> &second)
{
    double worst = 0.0;
    for(std::size_t at = 0; at < first.size() && at < second.size(); ++at)
        worst = std::max(worst, (first[at].block<3, 1>(0, 3) - second[at].block<3, 1>(0, 3)).norm());

    return worst;
}

// One stencil, the controls over it and the second window beside them, standing where a composition
// would have left the three. Both routes record what they were handed rather than the case deriving
// it again.
struct staged
{
    explicit staged(const rigid_motion::capabilities &motions)
            : staged(motions, opening())
    {
    }

    staged(const rigid_motion::capabilities &motions, const rigid_motion::two_pose_window::settings &chosen)
            : invocations(0)
            , decoupled_invocations(0)
            , travelled()
            , decoupled_travelled()
            , scene()
            , body(scene, five_objects(), motions.frame)
            , panel("Two poses", body, motions, recording_into(invocations, travelled), chosen)
            , decoupled(
                      "Decoupled path", motions, [this] { return std::pair<transform, transform>{panel.start_pose(), panel.end_pose()}; },
                      recording_into(decoupled_invocations, decoupled_travelled))
    {
        REQUIRE(body.initialize().has_value());
    }

    staged(const staged &) = delete;

    void open()
    {
        panel.initialize();
        decoupled.initialize();
    }

    transform placed() const
    {
        return body.pose(rigid_motion::two_pose_window::body_object);
    }

    void idle(int frames_drawn)
    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);
        frames.draw([this] { panel.render(); }, frames_drawn);
    }

    std::size_t controls()
    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);

        return navigable_items(frames, [this] { panel.render(); });
    }

    std::size_t invocations;
    std::size_t decoupled_invocations;
    std::vector<transform> travelled;
    std::vector<transform> decoupled_travelled;
    threepp::Scene scene;
    rigid_motion::frame_stencil body;
    rigid_motion::two_pose_window panel;
    rigid_motion::decoupled_path_window decoupled;
};

inline bool posed_alike(const rigid_motion::two_pose_window &panel, const std::pair<transform, transform> &before)
{
    return panel.start_pose().cwiseEqual(before.first).all() && panel.end_pose().cwiseEqual(before.second).all();
}

// Every control the panel offers is driven in turn and answered for, so which of them move a pose
// comes from the panel. A press that moved nothing is as much of the assertion as one that did.
inline std::size_t controls_moving_a_pose(staged &built, std::size_t offered)
{
    std::size_t moved = 0;
    for(std::size_t step = 0; step < offered; ++step)
    {
        const std::pair<transform, transform> before{built.panel.start_pose(), built.panel.end_pose()};
        const std::size_t counted           = built.invocations;
        const std::function<bool()> unmoved = [&built, &before] { return posed_alike(built.panel, before); };

        drive_control_at([&built] { built.panel.render(); }, step, unmoved);

        const bool changed = !posed_alike(built.panel, before);
        moved += changed ? 1u : 0u;

        INFO(step);
        CHECK(built.invocations == counted + (changed ? 1u : 0u));
    }

    return moved;
}

}

#endif
