#include "praxis/optimization/curve_graph.h"

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <functional>

namespace praxis::optimization {

namespace {

constexpr const char *abscissa_axis = "x";
constexpr const char *ordinate_axis = "g(x)";

constexpr const char *nothing_published = "The function has not been evaluated yet.";

constexpr double unanswered = std::numeric_limits<double>::quiet_NaN();

struct linearized
{
    double x;
    double g;
    double slope;
};

double coordinate(const Eigen::VectorXd &x)
{
    return x.size() == 1 ? x[0] : unanswered;
}

// g = -e and g' = J; an answer the problem refused stands at NaN and is not drawn.
double value_of(const expected<Eigen::VectorXd, refusal> &e)
{
    return e && e->size() == 1 ? -(*e)[0] : unanswered;
}

double slope_of(const expected<Eigen::MatrixXd, refusal> &jacobian)
{
    return jacobian && jacobian->size() == 1 ? (*jacobian)(0, 0) : unanswered;
}

scene::plot_series iterate_marks(const scenario_run &run)
{
    scene::plot_series marks{"Iterates", {}, {}, scene::plot_mark::markers};
    for(std::size_t k = 0; k < run.iterates.size(); ++k)
    {
        marks.abscissa.push_back(coordinate(run.iterates[k]));
        marks.ordinate.push_back(k < run.errors.size() ? value_of(run.errors[k]) : unanswered);
    }

    return marks;
}

// The ordinates the plot is fitted to: those of finite points lying within the held range.
std::optional<interval> finite_span(const std::vector<scene::plot_series> &drawn, const interval &range)
{
    interval span{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for(const scene::plot_series &series : drawn)
        for(std::size_t at = 0; at < series.abscissa.size() && at < series.ordinate.size(); ++at)
            if(series.abscissa[at] >= range.lower && series.abscissa[at] <= range.upper && std::isfinite(series.ordinate[at]))
                span = interval{std::min(span.lower, series.ordinate[at]), std::max(span.upper, series.ordinate[at])};

    return span.lower <= span.upper ? std::optional<interval>(span) : std::nullopt;
}

// Held to the range and to the ordinates the plot is fitted to, so a steep line widens neither axis.
std::optional<scene::plot_series> tangent(std::string name, const linearized &at, const interval &range, const interval &span)
{
    if(!std::isfinite(at.g) || !std::isfinite(at.slope) || (at.slope == 0.0 && (at.g < span.lower || at.g > span.upper)))
        return std::nullopt;

    double from = range.lower;
    double to   = range.upper;
    if(at.slope != 0.0)
    {
        const double below = at.x + (span.lower - at.g) / at.slope;
        const double above = at.x + (span.upper - at.g) / at.slope;
        from               = std::max(from, std::min(below, above));
        to                 = std::min(to, std::max(below, above));
    }
    if(!(from < to))
        return std::nullopt;

    return scene::plot_series{std::move(name), {from, to}, {at.g + at.slope * (from - at.x), at.g + at.slope * (to - at.x)}, scene::plot_mark::line};
}

void add_step(const scenario_snapshot &shown, const interval &span, std::vector<scene::plot_series> &drawn)
{
    const scenario_run &run = shown.run;
    const std::size_t k     = shown.step;
    if(k >= run.iterates.size() || k >= run.errors.size() || k >= run.jacobians.size())
        return;

    const linearized at{coordinate(run.iterates[k]), value_of(run.errors[k]), slope_of(run.jacobians[k])};
    if(std::optional<scene::plot_series> line = tangent("tangent at x_k", at, shown.ranges.front(), span))
        drawn.push_back(std::move(*line));

    const double zero = at.slope != 0.0 ? at.x - at.g / at.slope : unanswered;
    if(std::isfinite(zero))
        drawn.push_back(scene::plot_series{"zero of the tangent", {zero}, {0.0}, scene::plot_mark::markers});

    if(k + 1 < run.iterates.size() && k + 1 < run.errors.size())
    {
        const double next = coordinate(run.iterates[k + 1]);
        drawn.push_back(scene::plot_series{"x_(k+1)", {next, next}, {0.0, value_of(run.errors[k + 1])}, scene::plot_mark::segments});
    }
}

// The point is drawn before the span is taken, so its tangents are held to an ordinate range holding it.
std::optional<linearized> linearized_point(const std::optional<scenario_linearization> &at, std::vector<scene::plot_series> &drawn)
{
    if(!at || at->point.size() != 1)
        return std::nullopt;

    const linearized here{at->point[0], value_of(at->error), slope_of(at->jacobian)};
    drawn.push_back(scene::plot_series{"p", {here.x}, {here.g}, scene::plot_mark::markers});

    return here;
}

void add_linearization(const std::optional<scenario_linearization> &at, const linearized &here, const interval &range, const interval &span, std::vector<scene::plot_series> &drawn)
{
    if(std::optional<scene::plot_series> line = tangent("tangent at p", here, range, span))
        drawn.push_back(std::move(*line));

    if(!at->check || !at->check->disagrees || at->check->central.size() != 1)
        return;

    if(std::optional<scene::plot_series> line = tangent("central difference at p", linearized{here.x, here.g, at->check->central(0, 0)}, range, span))
        drawn.push_back(std::move(*line));
}

void add_bounds(const std::optional<box> &bounds, const interval &span, std::vector<scene::plot_series> &drawn)
{
    if(!bounds || bounds->lower.size() != 1 || bounds->upper.size() != 1)
        return;

    scene::plot_series ends{"bounds", {}, {}, scene::plot_mark::segments};
    for(const double end : {bounds->lower[0], bounds->upper[0]})
        if(std::isfinite(end))
        {
            ends.abscissa.insert(ends.abscissa.end(), {end, end});
            ends.ordinate.insert(ends.ordinate.end(), {span.lower, span.upper});
        }

    if(!ends.abscissa.empty())
        drawn.push_back(std::move(ends));
}

}

scene::plot_reading curve_reading(const scenario_reader &seen)
{
    const std::shared_ptr<const scenario_snapshot> shown = seen.read();
    if(shown == nullptr)
        return scene::plot_reading{.message = nothing_published, .abscissa_label = abscissa_axis, .frames = {}};

    std::vector<scene::plot_series> drawn{scene::plot_series{ordinate_axis, shown->abscissa, shown->ordinate, scene::plot_mark::line}, iterate_marks(shown->run)};
    const std::optional<linearized> here = linearized_point(shown->linearization, drawn);
    if(const std::optional<interval> span = finite_span(drawn, shown->ranges.front()))
    {
        add_step(*shown, *span, drawn);
        add_bounds(shown->bounds, *span, drawn);
        if(here)
            add_linearization(shown->linearization, *here, shown->ranges.front(), *span, drawn);
    }

    scene::plot_frame frame{.ordinate_label       = ordinate_axis,
                            .logarithmic_ordinate = false,
                            .series               = std::move(drawn),
                            .abscissa_limits      = scene::plot_limits{shown->ranges.front().lower, shown->ranges.front().upper}};

    return scene::plot_reading{.message = std::string(), .abscissa_label = abscissa_axis, .frames = {std::move(frame)}};
}

std::shared_ptr<scene::plot_window> curve_graph(const scenario_window_inputs &inputs, std::string name)
{
    scene::plot_pick seeding = [scenario = inputs.scenario](std::size_t, double abscissa, double)
    { command(scenario, [abscissa](scenario_settings &settings) { settings.seed = Eigen::VectorXd::Constant(1, abscissa); }); };

    return std::make_shared<scene::plot_window>(std::move(name), std::function<void()>(), [seen = inputs.seen] { return curve_reading(seen); }, std::move(seeding));
}

}
