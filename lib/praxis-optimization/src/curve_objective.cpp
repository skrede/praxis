#include "curve_overlays.h"
#include "curve_objective.h"
#include "scenario_answers.h"
#include "scenario_defaults.h"

#include "praxis/optimization/slots.h"

#include <Eigen/Core>

#include <cmath>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::optimization {

namespace {

// Nocedal & Wright, Numerical Optimization, 1st ed., eq. (2.13): the quadratic model of f at x_k.
struct quadratic_model
{
    double x;
    double f;
    double slope;
    double curvature;

    double operator()(double at) const
    {
        return f + slope * (at - x) + 0.5 * curvature * (at - x) * (at - x);
    }
};

double answered(const expected<double, refusal> &f)
{
    return f ? *f : unanswered;
}

double first_of(const expected<Eigen::VectorXd, refusal> &gradient)
{
    return gradient && gradient->size() == 1 ? (*gradient)[0] : unanswered;
}

bool stepped_from(const scenario_run &run, std::size_t k)
{
    return k + 1 < run.iterates.size() && k < run.objectives.size() && k < run.gradients.size();
}

// Present where x_(k+1) was recorded and f, f' and f'' were answered finitely at x_k.
std::optional<quadratic_model> newton_model(const scenario_snapshot &shown)
{
    const scenario_run &run = shown.run;
    const std::size_t k     = shown.step;
    if(run.minimized_by != minimizer_slot::newton || !stepped_from(run, k) || k >= run.hessians.size())
        return std::nullopt;

    const quadratic_model q{coordinate(run.iterates[k]), answered(run.objectives[k]), first_of(run.gradients[k]), slope_of(run.hessians[k])};
    if(!std::isfinite(q.x) || !std::isfinite(q.f) || !std::isfinite(q.slope) || !std::isfinite(q.curvature))
        return std::nullopt;

    return q;
}

template<typename Model>
void add_next(const scenario_run &run, std::size_t k, const Model &model, std::vector<scene::plot_series> &drawn)
{
    const double next   = coordinate(run.iterates[k + 1]);
    const double landed = k + 1 < run.objectives.size() ? answered(run.objectives[k + 1]) : unanswered;
    drawn.push_back(scene::plot_series{"x_(k+1)", {next, next}, {model(next), landed}, scene::plot_mark::segments, iterate_tone});
}

}

void add_vertex(const scenario_snapshot &shown, std::vector<scene::plot_series> &drawn)
{
    const std::optional<quadratic_model> q = newton_model(shown);
    if(!q || q->curvature == 0.0)
        return;

    const double vertex = q->x - q->slope / q->curvature;
    if(std::isfinite(vertex) && std::isfinite((*q)(vertex)))
        drawn.push_back(scene::plot_series{"vertex", {vertex}, {(*q)(vertex)}, scene::plot_mark::markers, tangent_zero_tone});
}

void add_newton_model(const scenario_snapshot &shown, const interval &span, std::vector<scene::plot_series> &drawn)
{
    const std::optional<quadratic_model> q = newton_model(shown);
    if(!q)
        return;

    scene::plot_series model{"quadratic model at x_k", {}, {}, scene::plot_mark::line, tangent_tone};
    for(std::size_t at = 0; at < curve_samples; ++at)
    {
        const double x = evenly(shown.ranges.front(), at, curve_samples);
        const double y = (*q)(x);
        model.abscissa.push_back(x);
        model.ordinate.push_back(y >= span.lower && y <= span.upper ? y : unanswered);
    }
    drawn.push_back(std::move(model));
    add_next(shown.run, shown.step, *q, drawn);
}

void add_descent_tangent(const scenario_snapshot &shown, const interval &span, std::vector<scene::plot_series> &drawn)
{
    const scenario_run &run = shown.run;
    const std::size_t k     = shown.step;
    if(!stepped_from(run, k))
        return;

    const linearized at{coordinate(run.iterates[k]), answered(run.objectives[k]), first_of(run.gradients[k])};
    if(!std::isfinite(at.x) || !std::isfinite(at.g) || !std::isfinite(at.slope))
        return;

    if(std::optional<scene::plot_series> line = tangent("tangent at x_k", tangent_tone, at, shown.ranges.front(), span))
        drawn.push_back(std::move(*line));
    add_next(run, k, [&at](double x) { return at.g + at.slope * (x - at.x); }, drawn);
}

}
