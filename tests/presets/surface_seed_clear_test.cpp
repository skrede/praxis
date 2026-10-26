#include "figure_clearance.h"
#include "standing_windows.h"
#include "scenario_defaults.h"
#include "composed_optimization.h"

#include "scratch_documents.h"
#include "offered_optimization.h"

#include "praxis/optimization/registered.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>
#include <threepp/math/Matrix4.hpp>

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/InstancedMesh.hpp>

#include <threepp/cameras/PerspectiveCamera.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::fixture;

namespace {

struct frame_size
{
    float width;
    float height;
    float scale;
};

constexpr std::array<frame_size, 2> frames{{{1920.f, 1080.f, 1.f}, {2560.f, 1440.f, 2.f}}};

std::vector<optimization::registered_surface> shipped_surfaces()
{
    return {optimization::registered_surface(
                    "x^2 + y^2 - 1", [](double x, double y) { return x * x + y * y - 1.0; }, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); }),
            optimization::registered_surface(
                    "x y - 1", [](double x, double y) { return x * y - 1.0; }, [](double x, double y) { return Eigen::Vector2d(y, x); }),
            optimization::registered_surface(
                    "(x^2 + 10y^2)/2", [](double x, double y) { return 0.5 * (x * x + 10.0 * y * y); }, [](double x, double y) { return Eigen::Vector2d(x, 10.0 * y); },
                    [](double, double) { return Eigen::Matrix2d(Eigen::Vector2d(1.0, 10.0).asDiagonal()); })};
}

// The windows a fresh layout opens, in a context styled as the demonstration styles its own at a scale.
std::vector<standing_window> windows_over(const composed_optimization &composed, const frame_size &frame)
{
    const scratch_documents::scratch where("surface-seed-clear");
    const std::vector<std::string> presets = offered_optimization::offered(where).names;

    tests::imgui_frame context;
    style_as_shown(ImVec2(frame.width, frame.height), frame.scale);

    std::vector<standing_window> opened;
    context.draw_frame(
            [&composed, &presets, &opened]
            {
                const ImGuiViewport &viewport = *ImGui::GetMainViewport();
                opened                        = standing_windows(presets);
                for(const std::shared_ptr<scene::imgui_window> &window : composed.preset->windows)
                {
                    REQUIRE(window->opening().has_value());
                    const scene::window_rectangle at = scene::opening_rectangle(*window->opening(), viewport.WorkPos, viewport.WorkSize, ImGui::GetFontSize());
                    opened.push_back({window->display_name(), at.position, at.size});
                }
            });

    return opened;
}

// The opening view's 75 degree camera, near 0.1 m and far 100 m.
std::shared_ptr<threepp::PerspectiveCamera> opening_camera(const composed_optimization &composed, const frame_size &frame)
{
    const std::optional<scene::camera_view> &view = composed.preset->opening_view;
    REQUIRE(view.has_value());
    auto camera = threepp::PerspectiveCamera::create(75.f, frame.width / frame.height, 0.1f, 100.f);
    camera->position.copy(view->position);
    camera->lookAt(view->target);
    camera->updateMatrixWorld(true);
    composed.scene->updateMatrixWorld(true);

    return camera;
}

Eigen::Vector2d pixel_of(threepp::Vector3 world, const threepp::Camera &camera, const frame_size &frame)
{
    world.project(camera);

    return Eigen::Vector2d((world.x + 1.0) / 2.0 * frame.width, (1.0 - world.y) / 2.0 * frame.height);
}

// x_0's marker: the selected step's is drawn last, and a run opens at step 0.
disc seed_on(const composed_optimization &composed, const threepp::Camera &camera, const frame_size &frame)
{
    const auto *marks = composed.scene->getObjectByName("iterates")->children.at(0)->as<threepp::InstancedMesh>();
    REQUIRE(marks != nullptr);
    threepp::Matrix4 placed;
    marks->getMatrixAt(marks->count() - 1u, placed);
    threepp::Vector3 center(placed.elements[12], placed.elements[13], placed.elements[14]);
    center.applyMatrix4(*marks->matrixWorld);

    threepp::Vector3 up;
    up.setFromMatrixColumn(*camera.matrixWorld, 1);
    const Eigen::Vector2d middle = pixel_of(center, camera, frame);

    return disc{middle, (pixel_of(center.clone().addScaledVector(up, static_cast<float>(optimization::iterate_radius)), camera, frame) - middle).norm()};
}

// Every vertex of the gradient arrow's head, its cone's tip and base rim, where the camera draws it.
std::vector<Eigen::Vector2d> head_on(const composed_optimization &composed, const threepp::Camera &camera, const frame_size &frame)
{
    threepp::Object3D *arrow = composed.scene->getObjectByName("gradient at x_k");
    REQUIRE(arrow != nullptr);
    const auto *head = arrow->getObjectByName("head")->as<threepp::Mesh>();
    for(const threepp::Object3D *at = head; at != nullptr; at = at->parent)
        REQUIRE(at->visible);

    const auto *vertices = head->geometry()->getAttribute<float>("position");
    std::vector<Eigen::Vector2d> drawn;
    for(std::size_t vertex = 0; vertex < static_cast<std::size_t>(vertices->count()); ++vertex)
        drawn.push_back(pixel_of(threepp::Vector3(vertices->getX(vertex), vertices->getY(vertex), vertices->getZ(vertex)).applyMatrix4(*head->matrixWorld), camera, frame));

    return drawn;
}

template<typename projecting>
void check_clear(const char *document, const std::string &preset, const char *figure, projecting &&projected)
{
    composed_optimization composed(config::resolve(document, PRAXIS_SHIPPED_OPTIMIZATION_DIR), {}, preset, shipped_surfaces());
    composed.preset->stencil->render();
    for(const frame_size &frame : frames)
    {
        const auto camera = opening_camera(composed, frame);
        const auto drawn  = projected(composed, *camera, frame);
        for(const standing_window &window : windows_over(composed, frame))
        {
            const double clear = clearance(drawn, window);
            INFO(document << " at " << frame.width << " x " << frame.height << ": " << figure << " stands " << clear << " px clear of " << window.name);
            CHECK(clear > 0.0);
        }
    }
}

}

TEST_CASE("the root paraboloid opens with its seed marker clear of every window at both scales", "[presets][optimization][view]")
{
    check_clear("surface.xml", "Root finding: a paraboloid", "the seed", &seed_on);
}

TEST_CASE("the typed paraboloid opens with its seed marker clear of every window at both scales", "[presets][optimization][view]")
{
    check_clear("surface-typed.xml", "Typed: x^2 + ay^2 - 1", "the seed", &seed_on);
}

TEST_CASE("the minimized elliptic paraboloid opens with its seed marker clear of every window at both scales", "[presets][optimization][view]")
{
    check_clear("surface-minimum.xml", "Minimization: (x^2 + 10y^2)/2", "the seed", &seed_on);
}

TEST_CASE("the minimized elliptic paraboloid opens with its gradient arrow's head clear of every window at both scales", "[presets][optimization][view]")
{
    check_clear("surface-minimum.xml", "Minimization: (x^2 + 10y^2)/2", "the gradient arrow's head", &head_on);
}

TEST_CASE("a surface opens its windows apart from each other and from the standing windows at both scales", "[presets][optimization][view]")
{
    const composed_optimization composed(config::resolve("surface-minimum.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR), {}, "Minimization: (x^2 + 10y^2)/2", shipped_surfaces());
    for(const frame_size &frame : frames)
    {
        const std::vector<standing_window> opened = windows_over(composed, frame);
        for(std::size_t one = 0; one < opened.size(); ++one)
            for(std::size_t other = one + 1u; other < opened.size(); ++other)
            {
                INFO(frame.width << " x " << frame.height << ": " << opened[one].name << " and " << opened[other].name);
                if(opened[one].name.ends_with("##surface") || opened[other].name.ends_with("##surface"))
                    CHECK_FALSE(overlap(opened[one], opened[other]));
            }
    }
}
