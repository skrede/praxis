#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_SCENARIO_PLACEMENTS_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_SCENARIO_PLACEMENTS_H

#include "scratch_directory.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"

#include <array>
#include <string>
#include <fstream>
#include <filesystem>
#include <string_view>

namespace praxis::fixture {

inline constexpr const char *placed_curve_document = "<optimization><preset name=\"placed curve\" scenario=\"curve\"/><function source=\"manual\" expression=\"x^3 - 2x - 5\"/>"
                                                     "<range x_lower=\"-3\" x_upper=\"3\"/><seed x=\"1\"/></optimization>";

inline constexpr const char *placed_surface_document = "<optimization><preset name=\"placed surface\" scenario=\"surface\"/><function source=\"manual\" expression=\"x^2 + y^2 - 1\"/>"
                                                       "<range x_lower=\"-2\" x_upper=\"2\" y_lower=\"-2\" y_upper=\"2\"/><seed x=\"-2\" y=\"-2\"/></optimization>";

// A window's name as the GUI library keys it, the title it shows, and where it opens first.
struct pinned_placement
{
    std::string_view name;
    std::string_view title;
    scene::opening_placement at;
};

inline constexpr std::array<pinned_placement, 5> curve_placements{{
        {"Function##curve", "Function", {scene::window_corner::top_left, 1.f, 10.f, 35.f, 15.f}},
        {"Graph##curve", "Graph", {scene::window_corner::top_left, 37.f, 16.f, 23.f, 23.f}},
        {"Display##curve", "Display", {scene::window_corner::top_left, 37.f, 1.f, 12.f, 10.f}},
        {"Solve##curve", "Solve", {scene::window_corner::bottom_right, 1.f, 20.f, 36.f, 19.f}},
        {"Linearization##curve", "Linearization", {scene::window_corner::bottom_right, 1.f, 1.f, 39.f, 14.f}},
}};

inline constexpr std::array<pinned_placement, 4> surface_placements{{
        {"Function##surface", "Function", {scene::window_corner::top_left, 1.f, 10.f, 35.f, 19.f}},
        {"Display##surface", "Display", {scene::window_corner::top_left, 37.f, 1.f, 12.f, 15.f}},
        {"Solve##surface", "Solve", {scene::window_corner::bottom_right, 1.f, 20.f, 36.f, 19.f}},
        {"Linearization##surface", "Linearization", {scene::window_corner::bottom_right, 1.f, 1.f, 39.f, 18.f}},
}};

inline config::location placed_document(const std::string &file, const char *body)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization_window_placement";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

inline bool same(const scene::opening_placement &one, const scene::opening_placement &other)
{
    return one.corner == other.corner && one.inset_x == other.inset_x && one.inset_y == other.inset_y && one.width == other.width && one.height == other.height;
}

}

#endif
