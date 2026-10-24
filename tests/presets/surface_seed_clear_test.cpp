#include "scenario_defaults.h"
#include "composed_optimization.h"

#include "praxis/optimization/registered.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>
#include <threepp/math/Matrix4.hpp>

#include <threepp/objects/InstancedMesh.hpp>

#include <threepp/cameras/PerspectiveCamera.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <optional>
#include <algorithm>

#ifndef PRAXIS_SHIPPED_OPTIMIZATION_DIR
    #define PRAXIS_SHIPPED_OPTIMIZATION_DIR ""
#endif

using namespace praxis;
using namespace praxis::fixture;

namespace {

// The font size at font scale 1, in px.
constexpr float unit_at_scale_one = 13.f;

// A frame the demonstration opens, in px, and the preset selector's size on its first use there.
struct frame_size
{
    float width;
    float height;
    float scale;
    float selector_width;
    float selector_height;
};

constexpr std::array<frame_size, 2> frames{{{1920.f, 1080.f, 1.f, 392.f, 54.f}, {2560.f, 1440.f, 2.f, 785.f, 108.f}}};

struct named_rectangle
{
    std::string name;
    ImVec2 position;
    ImVec2 size;
};

struct disc
{
    Eigen::Vector2d center;
    double radius;
};

double paraboloid(double x, double y)
{
    return x * x + y * y - 1.0;
}

double hyperbolic_paraboloid(double x, double y)
{
    return x * y - 1.0;
}

std::vector<optimization::registered_surface> shipped_surfaces()
{
    return {optimization::registered_surface("x^2 + y^2 - 1", &paraboloid, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); }),
            optimization::registered_surface("x y - 1", &hyperbolic_paraboloid, [](double x, double y) { return Eigen::Vector2d(y, x); }),
            optimization::registered_surface(
                    "(x^2 + 10y^2)/2", [](double x, double y) { return 0.5 * (x * x + 10.0 * y * y); }, [](double x, double y) { return Eigen::Vector2d(x, 10.0 * y); },
                    [](double, double) { return Eigen::Matrix2d(Eigen::Vector2d(1.0, 10.0).asDiagonal()); })};
}

// The windows a fresh layout opens over the work area, which is the whole frame.
std::vector<named_rectangle> windows_over(const composed_optimization &composed, const frame_size &frame)
{
    const float s = frame.scale, w = frame.width, h = frame.height;
    std::vector<named_rectangle> standing{{"Messages", {20.f, h - 20.f - 0.25f * h}, {0.45f * w, 0.25f * h}},
                                          {"Stepped work", {w - 20.f - 0.45f * w, 20.f}, {0.45f * w, 0.25f * h}},
                                          {"view gizmo", {w - 108.f * s, 12.f * s}, {96.f * s, 96.f * s}},
                                          {"Presets", {60.f, 60.f}, {frame.selector_width, frame.selector_height}}};
    for(const std::shared_ptr<scene::imgui_window> &window : composed.preset->windows)
    {
        REQUIRE(window->opening().has_value());
        const scene::window_rectangle at = scene::opening_rectangle(*window->opening(), ImVec2(0.f, 0.f), ImVec2(w, h), unit_at_scale_one * s);
        standing.push_back({window->display_name(), at.position, at.size});
    }

    return standing;
}

Eigen::Vector2d pixel_of(threepp::Vector3 world, const threepp::Camera &camera, const frame_size &frame)
{
    world.project(camera);

    return Eigen::Vector2d((world.x + 1.0) / 2.0 * frame.width, (1.0 - world.y) / 2.0 * frame.height);
}

// x_0's marker as the opening view's 75 degree camera (near 0.1 m, far 100 m) draws it on the frame. The
// selected step's marker is drawn last, and a run opens at step 0.
disc seed_on(composed_optimization &composed, const frame_size &frame)
{
    const std::optional<scene::camera_view> &view = composed.preset->opening_view;
    REQUIRE(view.has_value());
    const auto camera = threepp::PerspectiveCamera::create(75.f, frame.width / frame.height, 0.1f, 100.f);
    camera->position.copy(view->position);
    camera->lookAt(view->target);
    camera->updateMatrixWorld(true);

    composed.scene->updateMatrixWorld(true);
    const auto *marks = composed.scene->getObjectByName("iterates")->children.at(0)->as<threepp::InstancedMesh>();
    REQUIRE(marks != nullptr);
    threepp::Matrix4 placed;
    marks->getMatrixAt(marks->count() - 1u, placed);
    threepp::Vector3 center(placed.elements[12], placed.elements[13], placed.elements[14]);
    center.applyMatrix4(*marks->matrixWorld);

    threepp::Vector3 up;
    up.setFromMatrixColumn(*camera->matrixWorld, 1);
    const Eigen::Vector2d middle = pixel_of(center, *camera, frame);

    return disc{middle, (pixel_of(center.clone().addScaledVector(up, static_cast<float>(optimization::iterate_radius)), *camera, frame) - middle).norm()};
}

// How far the disc stands outside the rectangle, in px; negative where the two overlap.
double clearance(const disc &seed, const named_rectangle &window)
{
    const Eigen::Vector2d low(window.position.x, window.position.y);
    const Eigen::Vector2d high = low + Eigen::Vector2d(window.size.x, window.size.y);
    const Eigen::Vector2d nearest(std::clamp(seed.center.x(), low.x(), high.x()), std::clamp(seed.center.y(), low.y(), high.y()));
    if(nearest != seed.center)
        return (seed.center - nearest).norm() - seed.radius;

    return -std::min({seed.center.x() - low.x(), high.x() - seed.center.x(), seed.center.y() - low.y(), high.y() - seed.center.y()}) - seed.radius;
}

void check_seed_clear(const char *document, const std::string &preset)
{
    composed_optimization composed(config::resolve(document, PRAXIS_SHIPPED_OPTIMIZATION_DIR), {}, preset, shipped_surfaces());
    composed.preset->stencil->render();
    for(const frame_size &frame : frames)
    {
        const disc seed = seed_on(composed, frame);
        for(const named_rectangle &window : windows_over(composed, frame))
        {
            const double clear = clearance(seed, window);
            INFO(document << " at " << frame.width << " x " << frame.height << ": the seed at (" << seed.center.x() << ", " << seed.center.y() << ") px stands " << clear
                          << " px clear of " << window.name);
            CHECK(clear > 0.0);
        }
    }
}

}

TEST_CASE("the root paraboloid opens with its seed marker clear of every window at both scales", "[presets][optimization][view]")
{
    check_seed_clear("surface.xml", "Root finding: a paraboloid");
}

TEST_CASE("the typed paraboloid opens with its seed marker clear of every window at both scales", "[presets][optimization][view]")
{
    check_seed_clear("surface-typed.xml", "Typed: x^2 + ay^2 - 1");
}

TEST_CASE("the minimized elliptic paraboloid opens with its seed marker clear of every window at both scales", "[presets][optimization][view]")
{
    check_seed_clear("surface-minimum.xml", "Minimization: (x^2 + 10y^2)/2");
}
