#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_KEYED_SURFACE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_KEYED_SURFACE_H

#include "drawn_surface.h"
#include "composed_curve.h"
#include "scenario_defaults.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/surface_key.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/scenario_state.h"

#include "praxis/scene/key_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

namespace praxis::fixture {

// A surface composed headless with its key, drained and drawn once.
struct keyed_surface
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<scene::key_window> key;
    opened taken;

    keyed_surface(registered_surface surface, const scenario_settings &settings, bool checks_derivative)
            : loop(scheduler::inline_workers)
            , built(loop)
            , key()
            , taken(open_surface_with(built.site, {std::move(surface)}, baseline(), settings, composer(key), checks_derivative))
    {
        REQUIRE(settled(loop, taken) != nullptr);
        taken.composed->stencil->render();
    }

    ~keyed_surface()
    {
        taken.composed->tear_down();
    }

    static scenario_window_composer composer(std::shared_ptr<scene::key_window> &kept)
    {
        return [&kept](const scenario_window_inputs &offered)
        {
            kept = optimization::surface_key(offered, "Key");

            return std::vector<std::shared_ptr<scene::imgui_window>>{kept};
        };
    }

    threepp::Scene &scene()
    {
        return *built.scene;
    }

    const std::vector<scene::key_entry> &keyed() const
    {
        REQUIRE(taken.inputs->keyed != nullptr);

        return *taken.inputs->keyed;
    }

    // As the Linearization window's Show and the Display window's switches write the view.
    void linearized_at(const Eigen::Vector2d &p)
    {
        command(taken.inputs->scenario, [p](scenario_state &state) { state.linearize(p); });
        REQUIRE(loop.drain().has_value());
        set(&scenario_view::linearization, true);
    }

    void set(bool scenario_view::*part, bool shown)
    {
        (*taken.inputs->drawn).*part = shown;
        taken.composed->stencil->render();
    }
};

inline std::vector<std::string> labels_of(const std::vector<scene::key_entry> &entries)
{
    std::vector<std::string> labels;
    for(const scene::key_entry &entry : entries)
        labels.push_back(entry.label);

    return labels;
}

struct worn_figure
{
    std::string label;
    threepp::Color color;
};

inline constexpr std::array<std::pair<const char *, const char *>, 13> named_figures{{{"surface", "f(x, y)"},
                                                                                      {"zero curve", "zero curve"},
                                                                                      {"level curves", "level curves"},
                                                                                      {"tangent plane", "tangent at x_k"},
                                                                                      {"zero line", "zero line"},
                                                                                      {"nearest point", "nearest point"},
                                                                                      {"x_(k+1)", "x_(k+1)"},
                                                                                      {"gradient at x_k", "gradient at x_k"},
                                                                                      {"step taken", "step taken"},
                                                                                      {"box", "bounds"},
                                                                                      {"linearization point p", "p"},
                                                                                      {"tangent plane at p", "tangent at p"},
                                                                                      {"central difference at p", "difference at p"}}};

inline threepp::Color worn(threepp::Object3D &figure)
{
    const threepp::Object3D &toned = figure.material() != nullptr ? figure : *figure.children.front();
    const auto *material           = toned.materialAs<threepp::MaterialWithColor>();
    REQUIRE(material != nullptr);

    return material->color;
}

// Each distinct pair of key label and color among the marks under `name`, where they are shown.
inline void add_marks(threepp::Scene &scene, const char *name, std::array<const char *, 2> labels, std::uint32_t selected, std::vector<worn_figure> &shown)
{
    threepp::Object3D *group = scene.getObjectByName(name);
    if(!drawn_visible(scene, name) || group->children.empty())
        return;

    const auto *marks = group->children.front()->as<threepp::InstancedMesh>();
    REQUIRE(marks != nullptr);
    for(std::size_t k = 0; k < marks->count(); ++k)
    {
        const worn_figure mark{tone_at(*marks, k) == threepp::Color(selected) ? labels[1] : labels[0], tone_at(*marks, k)};
        if(std::ranges::none_of(shown, [&mark](const worn_figure &held) { return held.label == mark.label && held.color == mark.color; }))
            shown.push_back(mark);
    }
}

// What the scene graph shows, each figure named as the key names it, in the color it wears.
inline std::vector<worn_figure> shown_figures(threepp::Scene &scene)
{
    std::vector<worn_figure> shown;
    for(const auto &[name, label] : named_figures)
        if(drawn_visible(scene, name) && (std::string(name) != "surface" || field_of(scene).triangles > 0u))
            shown.push_back(worn_figure{label, worn(*scene.getObjectByName(name))});
    add_marks(scene, "iterates", {"Iterates", "x_k"}, selected_iterate_tone, shown);
    add_marks(scene, "iterates on the floor", {"floor iterates", "x_k on the floor"}, floor_selected_iterate_tone, shown);

    return shown;
}

}

#endif
