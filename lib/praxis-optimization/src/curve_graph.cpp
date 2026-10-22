#include "curve_overlays.h"
#include "scenario_answers.h"

#include "praxis/optimization/curve_graph.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::optimization {

namespace {

constexpr const char *abscissa_axis = "x";
constexpr const char *ordinate_axis = "g(x)";

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

std::vector<scene::plot_series> drawn_parts(const scenario_snapshot &shown, const scenario_view &view)
{
    std::vector<scene::plot_series> drawn{scene::plot_series{ordinate_axis, shown.abscissa, shown.ordinate, scene::plot_mark::line}};
    if(view.iterates)
        drawn.push_back(iterate_marks(shown.run));

    const std::optional<linearized> here = view.linearization ? linearized_point(shown.linearization, drawn) : std::nullopt;
    if(const std::optional<interval> span = finite_span(drawn, shown.ranges.front()))
    {
        if(view.step)
            add_step(shown, *span, drawn);
        if(view.bounds)
            add_bounds(shown.bounds, *span, drawn);
        if(here)
            add_linearization(shown.linearization, *here, shown.ranges.front(), *span, drawn);
    }

    return drawn;
}

}

scene::plot_reading curve_reading(const scenario_reader &seen, const scenario_view &drawn)
{
    const std::shared_ptr<const scenario_snapshot> shown = seen.read();
    if(shown == nullptr)
        return scene::plot_reading{.message = nothing_published, .abscissa_label = abscissa_axis, .frames = {}};

    scene::plot_frame frame{.ordinate_label       = ordinate_axis,
                            .logarithmic_ordinate = false,
                            .series               = drawn_parts(*shown, drawn),
                            .abscissa_limits      = scene::plot_limits{shown->ranges.front().lower, shown->ranges.front().upper}};

    return scene::plot_reading{.message = std::string(), .abscissa_label = abscissa_axis, .frames = {std::move(frame)}};
}

std::shared_ptr<scene::plot_window> curve_graph(const scenario_window_inputs &inputs, std::string name)
{
    scene::plot_pick seeding = [scenario = inputs.scenario](std::size_t, double abscissa, double)
    { command(scenario, [abscissa](scenario_settings &settings) { settings.seed = Eigen::VectorXd::Constant(1, abscissa); }); };

    scene::plot_source reading = [seen = inputs.seen, drawn = inputs.drawn != nullptr ? inputs.drawn : std::make_shared<scenario_view>()] { return curve_reading(seen, *drawn); };

    return std::make_shared<scene::plot_window>(std::move(name), std::function<void()>(), std::move(reading), std::move(seeding));
}

}
