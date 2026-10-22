#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_MINIMIZED_SURFACE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_MINIMIZED_SURFACE_H

#include "drawn_surface.h"
#include "composed_curve.h"

#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/display_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <threepp/math/Vector3.hpp>
#include <threepp/math/Matrix4.hpp>

#include <Eigen/Core>

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

namespace praxis::fixture {

inline registered_surface elliptic()
{
    return registered_surface(
            "(x^2 + 10y^2)/2", [](double x, double y) { return 0.5 * (x * x + 10.0 * y * y); }, [](double x, double y) { return Eigen::Vector2d(x, 10.0 * y); },
            [](double, double) { return Eigen::Matrix2d(Eigen::Vector2d(1.0, 10.0).asDiagonal()); });
}

inline scenario_settings shipped(problem_form form)
{
    return scenario_settings{
            "(x^2 + 10y^2)/2", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(-2.0, -2.0), 500u, 0u, objective_settings{form, minimizer_slot::newton, 0.1}, {}};
}

// The lowest of the field's drawn vertices, the floor's iterates in their order, and the direction the
// gradient arrow is turned onto, all in the field's frame.
inline double lowest_drawn(const drawn_field &field)
{
    double lowest = std::numeric_limits<double>::infinity();
    for(const unsigned int corner : field.corners)
        lowest = std::min(lowest, vertex(field, corner).z());

    return lowest;
}

inline std::vector<Eigen::Vector3d> projected_centers(threepp::Scene &scene)
{
    threepp::Object3D *projected = scene.getObjectByName("iterates on the floor");
    REQUIRE(projected != nullptr);
    REQUIRE(projected->children.size() == 1u);
    const auto *marks = projected->children.front()->as<threepp::InstancedMesh>();
    std::vector<Eigen::Vector3d> centers;
    threepp::Matrix4 placed;
    for(std::size_t k = 0; k < marks->count(); ++k)
    {
        marks->getMatrixAt(k, placed);
        centers.emplace_back(placed.elements[12], placed.elements[13], placed.elements[14]);
    }

    return centers;
}

inline Eigen::Vector3d pointing(threepp::Scene &scene)
{
    threepp::Vector3 along(0.f, 1.f, 0.f);
    along.applyQuaternion(scene.getObjectByName("gradient at x_k")->quaternion);

    return Eigen::Vector3d(along.x, along.y, along.z);
}

// One surface composed headless with the Display window kept, drained and drawn once.
struct minimized_surface
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<display_window> panel;
    opened taken;

    minimized_surface(registered_surface surface, const scenario_settings &settings, const capabilities &solving = baseline())
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken(open_surface_with(built.site, {std::move(surface)}, solving, settings, composer(panel)))
    {
        REQUIRE(settled(loop, taken) != nullptr);
        taken.composed->stencil->render();
    }

    explicit minimized_surface(problem_form form)
            : minimized_surface(elliptic(), shipped(form))
    {
    }

    ~minimized_surface()
    {
        taken.composed->tear_down();
    }

    static scenario_window_composer composer(std::shared_ptr<display_window> &kept)
    {
        return [&kept](const scenario_window_inputs &offered)
        {
            kept = std::make_shared<display_window>("Display", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{kept};
        };
    }

    threepp::Scene &scene()
    {
        return *built.scene;
    }

    std::shared_ptr<const scenario_snapshot> shown() const
    {
        return taken.inputs->seen.read();
    }

    void select(std::uint32_t step)
    {
        command(taken.inputs->scenario, [step](scenario_state &state) { state.select_step(step); });
        REQUIRE(loop.drain().has_value());
        taken.composed->stencil->render();
    }

    void press(const char *label)
    {
        tests::imgui_frame frames;
        press_on(frames, [shown = panel] { shown->render(); }, "Display", label);
        taken.composed->stencil->render();
    }

    // What the Display window writes when drawn once.
    std::string offered()
    {
        std::string drawn;
        tests::imgui_frame frames;
        frames.draw(
                [this, &drawn]
                {
                    ImGui::LogToBuffer();
                    panel->render();
                    drawn = GImGui->LogBuffer.c_str();
                    ImGui::LogFinish();
                });

        return drawn;
    }
};

}

#endif
