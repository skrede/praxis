#include "imgui_frame.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <implot_internal.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <functional>

using namespace praxis;

namespace {

using drawing = std::function<void()>;
using curves  = std::vector<scene::plot_series>;
using stacked = std::vector<scene::plot_frame>;

const char *const title               = "Plot";
const char *const absent_context_line = "No plotting context is current, so no curve is drawn.";

// The library names its plotting context through a single global, shared by every case in the
// executable. What this displaces is put back on scope exit, so a throw out of the swapped span
// cannot leave the name swapped for the cases that follow.
class named_context
{
public:
    explicit named_context(ImPlotContext *named)
            : m_displaced(ImPlot::GetCurrentContext())
    {
        ImPlot::SetCurrentContext(named);
    }

    named_context(const named_context &)            = delete;
    named_context(named_context &&)                 = delete;
    named_context &operator=(const named_context &) = delete;
    named_context &operator=(named_context &&)      = delete;

    ~named_context()
    {
        ImPlot::SetCurrentContext(m_displaced);
    }

private:
    ImPlotContext *m_displaced;
};

std::size_t geometry_of(const drawing &draw)
{
    tests::imgui_frame frames;
    frames.draw(draw);

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

// The panel a case expects, written out here rather than read off the window, so what is compared is
// every vertex the two drawings put on screen.
drawing panel_stating(const std::string &line)
{
    return [line]
    {
        ImGui::Begin(title);
        ImGui::TextUnformatted(line.c_str());
        ImGui::End();
    };
}

drawing panel_empty()
{
    return []
    {
        ImGui::Begin(title);
        ImGui::End();
    };
}

void draw_curve(const scene::plot_series &curve)
{
    const int count = static_cast<int>(std::min(curve.abscissa.size(), curve.ordinate.size()));
    ImPlotSpec spec;
    if(curve.mark == scene::plot_mark::markers)
    {
        spec.Marker = ImPlotMarker_Circle;
        ImPlot::PlotScatter(curve.name.c_str(), curve.abscissa.data(), curve.ordinate.data(), count, spec);
        return;
    }

    if(curve.mark == scene::plot_mark::segments)
        spec.Flags = ImPlotLineFlags_Segments;
    ImPlot::PlotLine(curve.name.c_str(), curve.abscissa.data(), curve.ordinate.data(), count, spec);
}

void draw_frame(const scene::plot_frame &shown, const std::string &abscissa_label, std::size_t index, float height, ImPlotLocation where)
{
    if(!ImPlot::BeginPlot(("##plot" + std::to_string(index)).c_str(), ImVec2(-1.f, height)))
        return;

    ImPlot::SetupAxes(abscissa_label.c_str(), shown.ordinate_label.c_str(), ImPlotAxisFlags_None,
                      shown.abscissa_limits ? ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit : ImPlotAxisFlags_None);
    if(shown.abscissa_limits)
        ImPlot::SetupAxisLimits(ImAxis_X1, shown.abscissa_limits->lower, shown.abscissa_limits->upper, ImPlotCond_Always);
    if(shown.logarithmic_ordinate)
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
    ImPlot::SetupLegend(where);

    for(const scene::plot_series &curve : shown.series)
        draw_curve(curve);

    ImPlot::EndPlot();
}

float stacked_height(std::size_t count)
{
    const float between = ImGui::GetStyle().ItemSpacing.y * static_cast<float>(count - 1u);

    return std::max((ImGui::GetContentRegionAvail().y - between) / static_cast<float>(count), ImPlot::GetStyle().PlotMinSize.y);
}

// Every frame is set up against the label the reading carries, save where a case names one of its
// own for a single frame to show that the reading's is what the window uses.
void draw_plot(const scene::plot_reading &shown, ImPlotLocation where, const std::vector<std::string> &instead = {})
{
    if(shown.frames.empty())
        return;

    const float height = stacked_height(shown.frames.size());
    for(std::size_t index = 0; index < shown.frames.size(); ++index)
        draw_frame(shown.frames[index], index < instead.size() ? instead[index] : shown.abscissa_label, index, height, where);
}

drawing panel_of(const scene::plot_reading &shown, drawing above = nullptr, ImPlotLocation where = ImPlotLocation_NorthWest, std::vector<std::string> instead = {})
{
    return [shown, where, instead = std::move(instead), above = std::move(above)]
    {
        ImGui::Begin(title);
        if(above)
            above();
        draw_plot(shown, where, instead);
        ImGui::End();
    };
}

scene::plot_source answering(scene::plot_reading given)
{
    return [given = std::move(given)] { return given; };
}

// Values single precision carries exactly, so a comparison against a hand-written panel is exact.
scene::plot_series curve_named(const std::string &name, double first)
{
    return scene::plot_series{name, {0.0, 1.0, 2.0, 3.0}, {first, first + 2.0, first + 4.0, first + 8.0}, scene::plot_mark::line};
}

scene::plot_frame frame_of(const std::string &label, double first)
{
    return scene::plot_frame{label, false, curves{curve_named(label, first)}, std::nullopt};
}

scene::plot_reading one_curve()
{
    return scene::plot_reading{"", "s", stacked{scene::plot_frame{"value", false, curves{curve_named("first", 1.0)}, std::nullopt}}};
}

// The top of the first plot's ordinate once the window has fitted it.
double fitted_top(const scene::plot_reading &shown)
{
    scene::plot_window panel(title, nullptr, answering(shown));
    tests::imgui_frame frames;
    frames.draw([&panel] { panel.render(); }, 4);

    return ImPlot::GetCurrentContext()->Plots.GetByIndex(0)->Axes[ImAxis_Y1].Range.Max;
}

scene::plot_reading two_curves()
{
    return scene::plot_reading{"", "s", stacked{scene::plot_frame{"value", false, curves{curve_named("first", 1.0), curve_named("second", 16.0)}, std::nullopt}}};
}

// One abscissa under three ordinates, which is the shape a reading of several frames exists for.
scene::plot_reading three_frames()
{
    return scene::plot_reading{"", "s", stacked{frame_of("first", 1.0), frame_of("second", 16.0), frame_of("third", 32.0)}};
}

// Four decades of ordinate, which is the span a logarithmic scale exists to show.
scene::plot_reading four_decades()
{
    const scene::plot_series falling{"residual", {0.0, 1.0, 2.0, 3.0, 4.0}, {1.0, 0.1, 0.01, 0.001, 0.0001}, scene::plot_mark::line};

    return scene::plot_reading{"", "iteration", stacked{scene::plot_frame{"error", true, curves{falling}, std::nullopt}}};
}

// The same four points under each mark, and a fifth and sixth repeating the last two so that a
// segment series of six covers the extents of four with one segment more.
scene::plot_reading marked(scene::plot_mark mark, std::size_t points = 4u)
{
    scene::plot_series curve = curve_named("first", 1.0);
    curve.mark               = mark;
    for(std::size_t extra = 4u; extra < points; ++extra)
    {
        curve.abscissa.push_back(curve.abscissa[extra - 2u]);
        curve.ordinate.push_back(curve.ordinate[extra - 2u]);
    }

    return scene::plot_reading{"", "s", stacked{scene::plot_frame{"value", false, curves{curve}, std::nullopt}}};
}

std::size_t vertices_of(const scene::plot_reading &shown)
{
    scene::plot_window panel(title, nullptr, answering(shown));
    tests::imgui_frame frames;
    frames.draw([&panel] { panel.render(); });

    return static_cast<std::size_t>(frames.vertices());
}

}

TEST_CASE("a plot window handed one curve puts vertices on screen", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(one_curve()));

    tests::imgui_frame frames;
    frames.draw([&panel] { panel.render(); });

    CHECK(frames.has_draw_data());
    CHECK(frames.vertices() > 0);
}

TEST_CASE("a plot window draws the axes, the labels and the points a hand-written panel does", "[scene]")
{
    const scene::plot_reading shown = one_curve();
    scene::plot_window panel(title, nullptr, answering(shown));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(shown)));
}

TEST_CASE("a plot window whose labels differ draws differently from one drawn without them", "[scene]")
{
    scene::plot_reading bare           = one_curve();
    bare.abscissa_label                = "";
    bare.frames.front().ordinate_label = "";
    scene::plot_window panel(title, nullptr, answering(one_curve()));

    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(bare)));
}

TEST_CASE("a plot window carrying a message draws the message and no plot at all", "[scene]")
{
    scene::plot_reading shown = one_curve();
    shown.message             = "Nothing to show.";
    scene::plot_window panel(title, nullptr, answering(shown));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_stating("Nothing to show.")));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(one_curve())));
}

TEST_CASE("a plot window carrying no frame at all draws neither a plot nor a message", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(scene::plot_reading{"", "s", stacked{}}));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_empty()));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(one_curve())));
}

TEST_CASE("the controls a plot window is given are drawn once a frame above its curves", "[scene]")
{
    const drawing stated   = [] { ImGui::TextUnformatted("Above"); };
    int invoked            = 0;
    const drawing counting = [&invoked, &stated]
    {
        ++invoked;
        stated();
    };
    scene::plot_window panel(title, counting, answering(one_curve()));

    {
        tests::imgui_frame frames;
        frames.draw([&panel] { panel.render(); });
    }
    CHECK(invoked == 2);

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(one_curve(), stated)));
}

TEST_CASE("a plot window given no source draws an empty panel rather than reading through nothing", "[scene]")
{
    scene::plot_window panel(title, nullptr, nullptr);

    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw([&panel] { panel.render(); });

    CHECK(frames.has_draw_data());
}

TEST_CASE("a curve whose two spans differ in length is drawn to the shorter of them", "[scene]")
{
    scene::plot_reading shown = one_curve();
    shown.frames.front().series.front().ordinate.pop_back();
    scene::plot_window panel(title, nullptr, answering(shown));

    scene::plot_reading shortened = shown;
    shortened.frames.front().series.front().abscissa.pop_back();

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(shortened)));
}

TEST_CASE("a reading carrying two curves draws both in one frame", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(two_curves()));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(two_curves())));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(one_curve())));
}

TEST_CASE("a reading carrying three frames draws three plots stacked in the order they stand", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(three_frames()));

    scene::plot_reading reversed = three_frames();
    std::reverse(reversed.frames.begin(), reversed.frames.end());

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(three_frames())));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(reversed)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(one_curve())));
}

TEST_CASE("each of three frames carries its own ordinate label and none of them its neighbor's", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(three_frames()));

    scene::plot_reading relabeled      = three_frames();
    relabeled.frames[1].ordinate_label = relabeled.frames[0].ordinate_label;

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(three_frames())));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(relabeled)));
}

TEST_CASE("every frame of one reading is set up against the one abscissa label the reading carries", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(three_frames()));

    const std::vector<std::string> shared{"s", "s", "s"};
    const std::vector<std::string> apart{"s", "step", "s"};

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(three_frames(), nullptr, ImPlotLocation_NorthWest, shared)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(three_frames(), nullptr, ImPlotLocation_NorthWest, apart)));
}

TEST_CASE("two curves carry a legend naming both, placed at the plot's upper left", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(two_curves()));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(two_curves(), nullptr, ImPlotLocation_NorthWest)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(two_curves(), nullptr, ImPlotLocation_SouthEast)));
}

TEST_CASE("a reading marked logarithmic draws differently from the same reading left linear", "[scene]")
{
    scene::plot_reading marked                 = two_curves();
    marked.frames.front().logarithmic_ordinate = true;
    scene::plot_window panel(title, nullptr, answering(marked));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(marked)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(two_curves())));
}

TEST_CASE("a frame turned logarithmic leaves the scales of the frames around it alone", "[scene]")
{
    scene::plot_reading middle            = three_frames();
    middle.frames[1].logarithmic_ordinate = true;
    scene::plot_window panel(title, nullptr, answering(middle));

    scene::plot_reading all = three_frames();
    for(scene::plot_frame &frame : all.frames)
        frame.logarithmic_ordinate = true;

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(middle)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(all)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(three_frames())));
}

TEST_CASE("a logarithmic ordinate spanning four decades draws without a reported fault", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(four_decades()));

    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw([&panel] { panel.render(); });

    CHECK(frames.has_draw_data());
    CHECK(frames.vertices() > 0);
}

TEST_CASE("a second plotting context leaves the first alive and current when it goes", "[scene]")
{
    const std::size_t expected = geometry_of(panel_of(one_curve()));
    REQUIRE(ImPlot::GetCurrentContext() == nullptr);

    scene::plot_window panel(title, nullptr, answering(one_curve()));

    {
        tests::imgui_frame frames;

        ImPlotContext *const standing = ImPlot::GetCurrentContext();
        REQUIRE(standing != nullptr);

        {
            const scene::plot_context inner;
            CHECK(ImPlot::GetCurrentContext() != nullptr);
            CHECK(ImPlot::GetCurrentContext() != standing);
        }
        CHECK(ImPlot::GetCurrentContext() == standing);

        frames.draw([&panel] { panel.render(); });
        CHECK(frames.vertices() > 0);
        CHECK(frames.signature() == expected);
    }

    CHECK(ImPlot::GetCurrentContext() == nullptr);
}

TEST_CASE("a plotting context destroyed while another is current leaves that other one current", "[scene]")
{
    tests::imgui_frame frames;

    ImPlotContext *const standing = ImPlot::GetCurrentContext();
    REQUIRE(standing != nullptr);

    // Both contexts are gone by the end of this case and neither of them names the suite's, so this
    // is what puts that one back for the cases that follow.
    const named_context held(standing);

    // Held so that the order the two are destroyed in is this case's to choose, not the scope's.
    std::optional<scene::plot_context> outer;
    std::optional<scene::plot_context> inner;

    outer.emplace();
    ImPlotContext *const made_outer = ImPlot::GetCurrentContext();
    REQUIRE(made_outer != standing);

    inner.emplace();
    ImPlotContext *const made_inner = ImPlot::GetCurrentContext();
    REQUIRE(made_inner != made_outer);

    // A context going away is not a reason to disturb one that is still current.
    outer.reset();
    CHECK(ImPlot::GetCurrentContext() == made_inner);

    inner.reset();

    // Neither of the two is left to name, and the one displaced by the first of them is the other.
    CHECK(ImPlot::GetCurrentContext() == nullptr);
}

TEST_CASE("a plot window drawn while no plotting context is current says so rather than reading through none", "[scene]")
{
    const std::size_t expected = geometry_of(panel_stating(absent_context_line));

    scene::plot_window panel(title, nullptr, answering(one_curve()));

    tests::imgui_frame frames;

    {
        const named_context none(nullptr);
        frames.draw([&panel] { panel.render(); });
    }

    CHECK(frames.has_draw_data());
    CHECK(frames.signature() == expected);
}

TEST_CASE("a series marked as markers draws a circle at each point and no line between them", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(marked(scene::plot_mark::markers)));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(marked(scene::plot_mark::markers))));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(marked(scene::plot_mark::line))));
}

TEST_CASE("a series of four points marked as segments draws two disjoint segments", "[scene]")
{
    scene::plot_window panel(title, nullptr, answering(marked(scene::plot_mark::segments)));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(marked(scene::plot_mark::segments))));
    CHECK(vertices_of(marked(scene::plot_mark::segments)) < vertices_of(marked(scene::plot_mark::line)));
    CHECK(vertices_of(marked(scene::plot_mark::segments, 6u)) == vertices_of(marked(scene::plot_mark::line)));
}

TEST_CASE("a line, a marker and a segment series draw together in one frame", "[scene]")
{
    scene::plot_reading together = marked(scene::plot_mark::line);
    for(const scene::plot_mark mark : {scene::plot_mark::markers, scene::plot_mark::segments})
    {
        scene::plot_series beside = marked(mark).frames.front().series.front();
        beside.name               = mark == scene::plot_mark::markers ? "markers" : "segments";
        together.frames.front().series.push_back(beside);
    }
    scene::plot_window panel(title, nullptr, answering(together));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(together)));
    CHECK(vertices_of(together) > vertices_of(marked(scene::plot_mark::line)));
}

TEST_CASE("a frame carrying abscissa limits holds its horizontal axis at them", "[scene]")
{
    scene::plot_reading held            = two_curves();
    held.frames.front().abscissa_limits = scene::plot_limits{-1.0, 7.0};
    scene::plot_window panel(title, nullptr, answering(held));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(held)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(two_curves())));
}

TEST_CASE("held limits and the ordinate fitted under them follow a reading that changes between frames", "[scene]")
{
    scene::plot_reading changed            = two_curves();
    changed.frames.front().abscissa_limits = scene::plot_limits{1.0, 2.0};
    for(double &value : changed.frames.front().series.front().ordinate)
        value *= 16.0;
    std::size_t expected = 0u;
    {
        tests::imgui_frame reference;
        reference.draw(panel_of(changed), 4);
        expected = reference.signature();
    }

    scene::plot_reading held            = two_curves();
    held.frames.front().abscissa_limits = scene::plot_limits{-1.0, 7.0};
    scene::plot_window panel(title, nullptr, [&held] { return held; });

    tests::imgui_frame frames;
    frames.draw([&panel] { panel.render(); });
    held = changed;
    frames.draw([&panel] { panel.render(); });

    CHECK(frames.signature() == expected);
}

TEST_CASE("held limits fit the ordinate to the points lying within them alone", "[scene]")
{
    scene::plot_reading inside            = two_curves();
    inside.frames.front().abscissa_limits = scene::plot_limits{-1.0, 7.0};
    scene::plot_reading beyond            = inside;
    beyond.frames.front().series.back().abscissa.push_back(100.0);
    beyond.frames.front().series.back().ordinate.push_back(1.0e6);

    CHECK(fitted_top(beyond) == fitted_top(inside));
    CHECK(fitted_top(inside) < 1.0e3);
}
