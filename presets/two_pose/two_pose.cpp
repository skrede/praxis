#include "praxis/presets/two_pose.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/two_pose_window.h"
#include "praxis/rigid_motion/decoupled_path_window.h"

#include "praxis/scene/imgui_window.h"

#include <Eigen/Core>

#include <threepp/objects/Line.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::presets {

namespace {

constexpr double body_edge = 0.30;

// The opening pair, in the frame the stencil's poses are written in: metres and degrees taken in
// the order named. The two paths it gives stand a fifth of the default view apart at their widest,
// so what the scenario is for is on screen before anything is touched.
rigid_motion::two_pose_window::settings opening()
{
    return rigid_motion::two_pose_window::settings{rigid_motion::two_pose_window::pose_controls{Eigen::Vector3f{0.6f, -0.5f, 0.3f}, Eigen::Vector3f::Zero(), axis_order::xyz},
                                                   rigid_motion::two_pose_window::pose_controls{Eigen::Vector3f{-0.6f, 0.5f, 0.9f}, Eigen::Vector3f{0.f, 90.f, 90.f}, axis_order::xyz},
                                                   0.f};
}

// Built where the path runs, in the frame the stencil's poses are written in: the object carrying
// the line is never placed, so what the line says about the path is what was built into it.
std::shared_ptr<threepp::Object3D> path_line(std::span<const transform> travelled, const threepp::Color &tone)
{
    std::vector<threepp::Vector3> points;
    points.reserve(travelled.size());
    for(const transform &put : travelled)
        points.emplace_back(static_cast<float>(put(0, 3)), static_cast<float>(put(1, 3)), static_cast<float>(put(2, 3)));

    auto drawn = threepp::BufferGeometry::create();
    drawn->setFromPoints(points);

    return threepp::Line::create(drawn, threepp::LineBasicMaterial::create({{"color", tone}}));
}

rigid_motion::two_pose_window::path_route path_into(rigid_motion::frame_stencil &target, std::size_t object, const threepp::Color &tone)
{
    return [&target, object, tone](std::span<const transform> travelled)
    {
        target.set_body(object,
                        travelled.empty() ? rigid_motion::object_body{rigid_motion::body_shape::none, 0.0, nullptr}
                                          : rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, path_line(travelled, tone)});
    };
}

std::vector<rigid_motion::stencil_object> two_pose_objects()
{
    const rigid_motion::object_body nothing{rigid_motion::body_shape::none, 0.0, nullptr};

    return {rigid_motion::stencil_object{"Start", rigid_motion::axes_settings{}, nothing}, rigid_motion::stencil_object{"End", rigid_motion::axes_settings{}, nothing},
            rigid_motion::stencil_object{"Body", rigid_motion::axes_settings{}, rigid_motion::object_body{rigid_motion::body_shape::cube, body_edge, nullptr}},
            rigid_motion::stencil_object{"Path", rigid_motion::axes_settings{false}, nothing},
            rigid_motion::stencil_object{"Decoupled path", rigid_motion::axes_settings{false}, nothing}};
}

}

std::shared_ptr<scene::preset> two_pose_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions)
{
    auto body = std::make_shared<rigid_motion::frame_stencil>(site.scene, two_pose_objects(), motions.frame);

    const auto driving = std::make_shared<rigid_motion::two_pose_window>("Two poses", *body, motions,
                                                                         path_into(*body, rigid_motion::two_pose_window::path_object, threepp::Color::darkviolet), opening());

    const std::vector<std::shared_ptr<scene::imgui_window>> windows{driving,
                                                                    std::make_shared<rigid_motion::decoupled_path_window>(
                                                                            "Decoupled path", motions,
                                                                            [driving] { return std::pair<transform, transform>{driving->start_pose(), driving->end_pose()}; },
                                                                            path_into(*body, rigid_motion::decoupled_path_window::path_object, threepp::Color::darkorange))};

    return std::make_shared<scene::preset>(body, windows, site.add_window, site.remove_window);
}

}
