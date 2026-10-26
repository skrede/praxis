#include "window_placements.h"

#include "praxis/scene/imgui_window.h"

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::presets {

namespace {

using scene::window_corner;

struct titled_placement
{
    std::string_view title;
    std::size_t dimension;
    scene::opening_placement at;
};

constexpr std::array<titled_placement, 10> placements{{
        {"Function", 1u, {window_corner::top_left, 1.f, 10.f, 35.f, 15.f}},
        {"Function", 2u, {window_corner::top_left, 1.f, 10.f, 35.f, 19.f}},
        {"Graph", 1u, {window_corner::top_left, 37.f, 16.f, 23.f, 23.f}},
        {"Display", 1u, {window_corner::top_left, 37.f, 1.f, 12.f, 10.f}},
        {"Display", 2u, {window_corner::top_left, 37.f, 1.f, 12.f, 15.f}},
        {"Solve", 1u, {window_corner::bottom_right, 1.f, 20.f, 36.f, 19.f}},
        {"Solve", 2u, {window_corner::bottom_right, 1.f, 20.f, 36.f, 19.f}},
        {"Linearization", 1u, {window_corner::bottom_right, 1.f, 1.f, 39.f, 14.f}},
        {"Linearization", 2u, {window_corner::bottom_right, 1.f, 1.f, 39.f, 18.f}},
        {"Key", 2u, {window_corner::bottom_right, 40.f, 0.f, 12.f, 17.f}},
}};

}

std::optional<scene::opening_placement> scenario_window_placement(std::string_view title, std::size_t dimension)
{
    for(const titled_placement &one : placements)
        if(one.title == title && one.dimension == dimension)
            return one.at;

    return std::nullopt;
}

}
