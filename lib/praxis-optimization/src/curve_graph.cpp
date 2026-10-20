#include "praxis/optimization/curve_graph.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <functional>

namespace praxis::optimization {

namespace {

constexpr const char *abscissa_axis = "x";
constexpr const char *ordinate_axis = "g(x)";

constexpr const char *nothing_published = "The function has not been evaluated yet.";

double coordinate(const Eigen::VectorXd &x)
{
    return x.size() == 1 ? x[0] : std::numeric_limits<double>::quiet_NaN();
}

// g = -e at each iterate; one whose error was refused stands at NaN and is not drawn.
scene::plot_series iterate_marks(const scenario_run &run)
{
    scene::plot_series marks{"Iterates", {}, {}, scene::plot_mark::markers};
    for(std::size_t k = 0; k < run.iterates.size(); ++k)
    {
        const bool answered = k < run.errors.size() && run.errors[k] && run.errors[k]->size() == 1;

        marks.abscissa.push_back(coordinate(run.iterates[k]));
        marks.ordinate.push_back(answered ? -(*run.errors[k])[0] : std::numeric_limits<double>::quiet_NaN());
    }

    return marks;
}

}

scene::plot_reading curve_reading(const scenario_reader &seen)
{
    const std::shared_ptr<const scenario_snapshot> shown = seen.read();
    if(shown == nullptr)
        return scene::plot_reading{.message = nothing_published, .abscissa_label = abscissa_axis, .frames = {}};

    scene::plot_frame drawn{.ordinate_label       = ordinate_axis,
                            .logarithmic_ordinate = false,
                            .series               = {scene::plot_series{ordinate_axis, shown->abscissa, shown->ordinate, scene::plot_mark::line}, iterate_marks(shown->run)},
                            .abscissa_limits      = scene::plot_limits{shown->range.lower, shown->range.upper}};

    return scene::plot_reading{.message = std::string(), .abscissa_label = abscissa_axis, .frames = {std::move(drawn)}};
}

std::shared_ptr<scene::plot_window> curve_graph(const scenario_window_inputs &inputs, std::string name)
{
    return std::make_shared<scene::plot_window>(std::move(name), std::function<void()>(), [seen = inputs.seen] { return curve_reading(seen); });
}

}
