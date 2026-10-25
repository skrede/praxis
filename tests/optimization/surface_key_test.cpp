#include "key_rows.h"
#include "keyed_surface.h"
#include "minimized_surface.h"
#include "scenario_defaults.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/scenario_state.h"

#include "praxis/scene/key_window.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>
#include <initializer_list>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr double unbounded = std::numeric_limits<double>::infinity();

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

registered_surface circle(double off)
{
    return registered_surface("circle", &circle_value, [off](double x, double y) { return Eigen::Vector2d(2.0 * x + off, 2.0 * y + off); });
}

registered_surface boxed_circle()
{
    return registered_surface(
            "circle", &circle_value, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); }, box{Eigen::Vector2d(-unbounded, -unbounded), Eigen::Vector2d(1.2, 1.5)});
}

enum class posed : std::uint8_t
{
    root,
    boxed,
    wrong_at_p,
    right_at_p,
    minimum,
    floored
};

std::unique_ptr<keyed_surface> composed(posed from)
{
    const scenario_settings corner{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(-2.0, -2.0), 70u, 0u, {}, {}};
    const bool at_p = from == posed::wrong_at_p || from == posed::right_at_p;
    auto keyed      = from == posed::minimum || from == posed::floored
            ? std::make_unique<keyed_surface>(elliptic(), elliptic_minimum(problem_form::minimum), false)
            : std::make_unique<keyed_surface>(from == posed::boxed ? boxed_circle() : circle(from == posed::wrong_at_p ? 1.0 : 0.0), corner, at_p);
    if(from == posed::floored)
        keyed->set(&scenario_view::floor_projection, true);
    if(at_p)
        keyed->linearized_at(Eigen::Vector2d::Zero());

    return keyed;
}

std::vector<std::string> opened_with(posed from)
{
    const std::vector<std::string> root{"f(x, y)", "Iterates", "x_k", "zero curve", "tangent plane at x_k", "zero line", "nearest point", "x_(k+1)"};
    const std::vector<std::string> minimum{"f(x, y)", "Iterates", "x_k", "level curves", "gradient at x_k", "step taken"};
    const auto with = [](std::vector<std::string> listed, std::initializer_list<std::string> added)
    {
        listed.insert(listed.end(), added);
        return listed;
    };
    const std::array<std::vector<std::string>, 6> opened{root,
                                                         with(root, {"bounds"}),
                                                         with(root, {"tangent plane at p", "central difference at p"}),
                                                         with(root, {"tangent plane at p"}),
                                                         minimum,
                                                         with(minimum, {"iterates on the floor", "x_k on the floor"})};

    return opened[static_cast<std::size_t>(from)];
}

// A composition, the Display switch then turned off, if any, and the entries that removes.
struct keyed_state
{
    posed from;
    bool scenario_view::*off;
    std::vector<std::string> removed;
};

std::vector<keyed_state> keyed_states()
{
    return {{posed::root, nullptr, {}},
            {posed::wrong_at_p, nullptr, {}},
            {posed::right_at_p, nullptr, {}},
            {posed::boxed, nullptr, {}},
            {posed::floored, nullptr, {}},
            {posed::boxed, &scenario_view::field, {"f(x, y)"}},
            {posed::boxed, &scenario_view::iterates, {"Iterates", "x_k"}},
            {posed::boxed, &scenario_view::zero_curve, {"zero curve"}},
            {posed::boxed, &scenario_view::step, {"tangent plane at x_k", "zero line", "nearest point", "x_(k+1)"}},
            {posed::boxed, &scenario_view::bounds, {"bounds"}},
            {posed::floored, &scenario_view::field, {"f(x, y)"}},
            {posed::floored, &scenario_view::iterates, {"Iterates", "x_k", "iterates on the floor", "x_k on the floor"}},
            {posed::floored, &scenario_view::level_curves, {"level curves"}},
            {posed::floored, &scenario_view::gradient, {"gradient at x_k"}},
            {posed::floored, &scenario_view::step, {"step taken"}}};
}

// Only the tangent's zero, its nearest point and x_(k+1) may share a tone.
void check_apart(const std::vector<scene::key_entry> &entries)
{
    const auto grouped = [](const std::string &label) { return label == "zero line" || label == "nearest point" || label == "x_(k+1)"; };
    for(std::size_t i = 0; i < entries.size(); ++i)
        for(std::size_t j = i + 1u; j < entries.size(); ++j)
            CHECK((entries[i].tone != entries[j].tone || (grouped(entries[i].label) && grouped(entries[j].label))));
}

void check_keyed(keyed_surface &keyed, const std::vector<std::string> &expected)
{
    const std::vector<scene::key_entry> &entries = keyed.keyed();
    CHECK(labels_of(entries) == expected);
    check_apart(entries);

    const std::vector<worn_figure> shown = shown_figures(keyed.scene());
    CHECK(shown.size() == entries.size());
    std::vector<ImU32> swatches;
    for(const scene::key_entry &entry : entries)
    {
        INFO(entry.label);
        const auto found = std::ranges::find_if(shown, [&entry](const worn_figure &figure) { return figure.label == entry.label; });
        REQUIRE(found != shown.end());
        CHECK(found->color == threepp::Color(entry.tone));
        swatches.push_back(swatch_of(entry.tone));
    }

    const drawn_key drawn = drawn_rows(*keyed.key);
    CHECK(drawn.lines == expected);
    CHECK(drawn.swatches == swatches);
}

}

TEST_CASE("a minimized surface's key names each figure the stencil drew, in order and in its tone, and draws them as rows", "[optimization][surface][key]")
{
    const std::unique_ptr<keyed_surface> keyed = composed(posed::minimum);

    const std::vector<std::uint32_t> toned{surface_tone, iterate_tone, selected_iterate_tone, level_curve_tone, gradient_tone, floor_step_tone};
    std::vector<std::uint32_t> worn_tones;
    for(const scene::key_entry &entry : keyed->keyed())
        worn_tones.push_back(entry.tone);
    CHECK(worn_tones == toned);
    check_keyed(*keyed, {"f(x, y)", "Iterates", "x_k", "level curves", "gradient at x_k", "step taken"});
}

TEST_CASE("in every state a surface's key names exactly the figures its scene graph shows, each Display switch removing its own", "[optimization][surface][key]")
{
    const std::vector<keyed_state> states = keyed_states();
    for(std::size_t at = 0; at < states.size(); ++at)
    {
        INFO("state " << at);
        const keyed_state &state                   = states[at];
        const std::unique_ptr<keyed_surface> keyed = composed(state.from);
        if(state.off != nullptr)
            keyed->set(state.off, false);
        std::vector<std::string> expected = opened_with(state.from);
        std::erase_if(expected, [&state](const std::string &label) { return std::ranges::find(state.removed, label) != state.removed.end(); });
        check_keyed(*keyed, expected);
    }
}
