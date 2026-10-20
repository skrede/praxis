#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_DRAWN_SURFACE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_DRAWN_SURFACE_H

#include "composed_curve.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/preset_site.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/BufferGeometry.hpp>
#include <threepp/core/BufferAttribute.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::fixture {

using namespace praxis::optimization;

struct drawn_field
{
    std::size_t triangles;
    std::vector<float> positions;
    std::vector<unsigned int> corners;
};

inline drawn_field field_of(threepp::Scene &scene)
{
    auto *mesh = scene.getObjectByName<threepp::Mesh>("surface");
    REQUIRE(mesh != nullptr);
    const std::shared_ptr<threepp::BufferGeometry> shaped = mesh->geometry();
    REQUIRE(shaped->getAttribute<float>("position") != nullptr);
    REQUIRE(shaped->getIndex() != nullptr);

    const std::vector<unsigned int> &index = shaped->getIndex()->array();
    const auto used                        = static_cast<std::size_t>(shaped->drawRange.count);
    REQUIRE(used <= index.size());

    return drawn_field{used / 3u, shaped->getAttribute<float>("position")->array(), std::vector<unsigned int>(index.begin(), index.begin() + static_cast<std::ptrdiff_t>(used))};
}

inline Eigen::Vector3d vertex(const drawn_field &field, std::size_t at)
{
    return Eigen::Vector3d(field.positions[3u * at], field.positions[3u * at + 1u], field.positions[3u * at + 2u]);
}

// The affine map the mesh's corners place the ranges by, and the vertical scale its tallest finite
// sample stands at, so a marker is checked against the drawn field rather than against constants.
struct placement
{
    Eigen::Vector3d origin;
    Eigen::Vector2d across;
    double scale;

    Eigen::Vector3d at(double x, double y, double f) const
    {
        return Eigen::Vector3d(origin.x() + across.x() * (x + 2.0), origin.y() + across.y() * (y + 2.0), scale * f);
    }
};

inline placement placement_of(const drawn_field &field, const scenario_grid &grid)
{
    const std::size_t n = grid.count;
    std::size_t tallest = 0u;
    for(std::size_t at = 0; at < grid.heights.size(); ++at)
        if(std::isfinite(grid.heights[at]) && std::abs(grid.heights[at]) > std::abs(grid.heights[tallest]))
            tallest = at;

    const Eigen::Vector3d first = vertex(field, 0u);
    const Eigen::Vector2d across((vertex(field, n - 1u).x() - first.x()) / 4.0, (vertex(field, (n - 1u) * n).y() - first.y()) / 4.0);

    return placement{first, across, vertex(field, tallest).z() / grid.heights[tallest]};
}

inline std::vector<Eigen::Vector3d> shown_markers(threepp::Scene &scene)
{
    threepp::Object3D *markers = scene.getObjectByName("iterates");
    REQUIRE(markers != nullptr);

    std::vector<Eigen::Vector3d> standing;
    for(const threepp::Object3D *marker : markers->children)
        if(marker->visible)
            standing.emplace_back(marker->position.x, marker->position.y, marker->position.z);

    return standing;
}

// The composer keeps what it was offered and builds the windows handed in, none by default.
inline opened open_surface_with(const scene::preset_site &site, std::vector<registered_surface> surfaces, const capabilities &solving, const scenario_settings &settings,
                                const scenario_window_composer &windows = {}, bool checks_derivative = false)
{
    opened taken;
    const scenario_window_composer kept = [&taken, &windows](const scenario_window_inputs &offered)
    {
        taken.inputs = offered;

        return windows ? windows(offered) : std::vector<std::shared_ptr<scene::imgui_window>>{};
    };
    taken.composed = compose_surface_scenario(site, std::move(surfaces), solving, settings, scenario_composition{kept, checks_derivative});

    return taken;
}

// Shown only where the object and every node above it are.
inline bool drawn_visible(threepp::Scene &scene, const char *name)
{
    const threepp::Object3D *found = scene.getObjectByName(name);
    REQUIRE(found != nullptr);
    for(; found != nullptr; found = found->parent)
        if(!found->visible)
            return false;

    return true;
}

inline Eigen::Vector3d drawn_position(threepp::Scene &scene, const char *name)
{
    const threepp::Object3D *found = scene.getObjectByName(name);
    REQUIRE(found != nullptr);

    return Eigen::Vector3d(found->position.x, found->position.y, found->position.z);
}

// The vertices within the drawn range of a geometry written point by point.
inline std::vector<Eigen::Vector3d> drawn_points(threepp::Scene &scene, const char *name)
{
    threepp::Object3D *found = scene.getObjectByName(name);
    REQUIRE(found != nullptr);
    REQUIRE(found->geometry() != nullptr);
    const auto *positions = found->geometry()->getAttribute<float>("position");
    REQUIRE(positions != nullptr);

    const auto used = std::min(static_cast<std::size_t>(std::max(found->geometry()->drawRange.count, 0)), positions->array().size() / 3u);
    std::vector<Eigen::Vector3d> points;
    for(std::size_t at = 0; at < used; ++at)
        points.emplace_back(positions->array()[3u * at], positions->array()[3u * at + 1u], positions->array()[3u * at + 2u]);

    return points;
}

}

#endif
