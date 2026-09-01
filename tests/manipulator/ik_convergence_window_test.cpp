#include "window_stage.h"

#include "praxis/manipulator/ik_convergence_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr const char *table_title = "Iterates";
constexpr const char *plot_title  = "Convergence";

// The straight line a geometrically falling error draws is a claim about the differences of the
// logarithms, and this is how far apart two of them may stand and still be one line.
constexpr double one_line = 1.0e-12;

const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
const rotation upright(rotation::Identity());

// An error that falls by a constant factor per step, which is what a solve converging at a fixed
// order leaves behind: the two errors fall by different factors so the curves are told apart.
std::vector<iteration_state> falling(std::uint32_t steps, double factor)
{
    std::vector<iteration_state> taken;
    double error = 1.0;
    for(std::uint32_t step = 0; step < steps; ++step)
    {
        taken.push_back(iteration_state{configuration(0.0, 0.0), error, 0.5 * error, error, step});
        error *= factor;
    }

    return taken;
}

std::shared_ptr<arm_publisher> carrying(const std::vector<std::vector<iteration_state>> &sequences)
{
    arm_snapshot seen = at_rest(configuration(0.0, 0.0), origin, upright);
    seen.iterations   = sequences;

    return publishing(seen);
}

scene::plot_series named(const scene::plot_reading &shown, const std::string &name)
{
    REQUIRE(shown.frames.size() == 1u);

    const std::vector<scene::plot_series> &drawn = shown.frames.front().series;
    const auto found                             = std::find_if(drawn.begin(), drawn.end(), [&name](const scene::plot_series &curve) { return curve.name == name; });

    REQUIRE(found != drawn.end());

    return *found;
}

// The differences of the logarithms of a curve's points, which are all equal exactly when the points
// stand on a straight line drawn on a logarithmic ordinate.
std::vector<double> log_slopes(const scene::plot_series &curve)
{
    std::vector<double> between;
    for(std::size_t point = 1; point < curve.ordinate.size(); ++point)
        between.push_back(std::log(curve.ordinate[point]) - std::log(curve.ordinate[point - 1]));

    return between;
}

// The panel a case expects, written out here rather than read off the window, so what is compared is
// every vertex the two put on screen: the two choices above, and the line where the curves would be.
// The panel drawn on its own, without the focus request: a request made every frame takes focus off
// the popup a list opens in and closes it before a key reaches it.
drawing over_alone(ik_iterate_window &panel)
{
    return [&panel] { panel.render(); };
}

drawing panel_stating(const char *line)
{
    return [line]
    {
        bool angular = true;
        bool linear  = true;

        ImGui::Begin(plot_title);
        ImGui::Checkbox("Angular error", &angular);
        ImGui::SameLine();
        ImGui::Checkbox("Linear error", &linear);
        ImGui::TextUnformatted(line);
        ImGui::End();
    };
}

}

TEST_CASE("the two errors of the chosen start are drawn per step on a logarithmic ordinate", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({falling(6u, 0.25)});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    const ik_convergence_window panel(plot_title, table);

    const scene::plot_reading shown = panel.reading();

    CHECK(shown.message.empty());
    CHECK(shown.abscissa_label == "Step");
    REQUIRE(shown.frames.size() == 1u);
    CHECK(shown.frames.front().logarithmic_ordinate);
    CHECK(shown.frames.front().ordinate_label == "Error");
    REQUIRE(shown.frames.front().series.size() == 2u);
    CHECK(named(shown, "Angular error").ordinate.size() == 6u);
    CHECK(named(shown, "Linear error").ordinate.size() == 6u);
    CHECK(named(shown, "Angular error").abscissa == std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0});
}

TEST_CASE("an error falling by a constant factor per step stands on a straight line on that ordinate", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({falling(8u, 0.125)});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    const ik_convergence_window panel(plot_title, table);

    const scene::plot_reading shown = panel.reading();

    REQUIRE(shown.frames.size() == 1u);
    for(const scene::plot_series &curve : shown.frames.front().series)
    {
        const std::vector<double> between = log_slopes(curve);

        REQUIRE(between.size() == 7u);
        for(const double slope : between)
            CHECK(std::abs(slope - between.front()) < one_line);
    }
}

TEST_CASE("a step whose error reached zero is left out of the curve and the steps around it are drawn", "[manipulator][convergence]")
{
    std::vector<iteration_state> steps = falling(4u, 0.25);
    steps[2].angular_error             = 0.0;

    const std::shared_ptr<arm_publisher> published = carrying({steps});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    const ik_convergence_window panel(plot_title, table);

    const scene::plot_reading shown  = panel.reading();
    const scene::plot_series angular = named(shown, "Angular error");

    REQUIRE(angular.ordinate.size() == 3u);
    CHECK(angular.abscissa == std::vector<double>{0.0, 1.0, 3.0});
    CHECK(std::none_of(angular.ordinate.begin(), angular.ordinate.end(), [](double value) { return value <= 0.0; }));
    CHECK(named(shown, "Linear error").ordinate.size() == 4u);
}

TEST_CASE("the two errors stand in one frame, and a reading that says something instead carries none", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({falling(5u, 0.25)});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    const ik_convergence_window drawn(plot_title, table);
    const ik_convergence_window silent(plot_title, table, ik_convergence_window::settings{false, false});

    REQUIRE(drawn.reading().frames.size() == 1u);
    CHECK(drawn.reading().frames.front().series.size() == 2u);
    CHECK(silent.reading().frames.empty());
}

TEST_CASE("a publication carrying no step says what to do in place of a plot", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    ik_convergence_window panel(plot_title, table);

    const scene::plot_reading shown = panel.reading();

    CHECK(shown.message == "Ask for a solve to see how its error falls.");
    CHECK(shown.frames.empty());
    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_stating("Ask for a solve to see how its error falls.")));
}

TEST_CASE("a chosen start that recorded no step says the same rather than drawing an empty axis", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({falling(4u, 0.25), {}});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>(), ik_iterate_window::settings{1u});
    const ik_convergence_window panel(plot_title, table);

    CHECK(panel.reading().message == "Ask for a solve to see how its error falls.");
    CHECK(panel.reading().frames.empty());
}

TEST_CASE("a plot told to draw neither error says which choice is missing", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({falling(4u, 0.25)});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    const ik_convergence_window panel(plot_title, table, ik_convergence_window::settings{false, false});

    CHECK(panel.reading().message == "Choose an error to draw.");
    CHECK(panel.reading().frames.empty());
}

TEST_CASE("the plot is about the start the table is about and follows it when that changes", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({{}, falling(7u, 0.25)});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    const ik_convergence_window panel(plot_title, table);

    REQUIRE(panel.reading().frames.empty());

    // The list of starts stands last in a panel whose chosen start drew no row, so the keyboard
    // reaches it from the end and the entry below the one it opens on is the other start.
    imgui_frame frames;
    const drawing draw = over_alone(table);
    start_navigating(frames, draw);
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);

    REQUIRE(table.selected().has_value());
    CHECK(*table.selected() == 1u);
    CHECK(named(panel.reading(), "Angular error").ordinate.size() == 7u);
}

TEST_CASE("the window carrying a key path offers its two choices to the document and one carrying none offers nothing", "[manipulator][convergence]")
{
    const std::shared_ptr<arm_publisher> published = carrying({falling(4u, 0.25)});
    ik_iterate_window table(table_title, published->reader(), std::weak_ptr<owned_arm>());
    const ik_convergence_window unnamed(plot_title, table);
    const ik_convergence_window named_at(plot_title, table, ik_convergence_window::settings{false, true}, "machine/ik_convergence");

    CHECK(unnamed.as_configurable() == nullptr);
    CHECK(named_at.as_configurable() == &named_at);
    CHECK(named_at.settings_path() == "machine/ik_convergence");
    CHECK_FALSE(named_at.state().angular);
    CHECK(named_at.state().linear);
}
