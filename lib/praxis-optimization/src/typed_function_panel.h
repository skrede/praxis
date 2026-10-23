#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_FUNCTION_PANEL_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_FUNCTION_PANEL_H

#include "typed/expression.h"

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::optimization {

inline constexpr const char *unaskable_range = "The minimum and the maximum must be finite, and the minimum must lie below the maximum.";

// One row per parameter of the latest publication: a slider over the parameter's range, and the
// range's minimum and maximum, asked for once a field is left after an edit. A row keeps the values
// it shows while it is being edited or a value of it waits to be asked for.
class parameter_rows
{
    struct parameter_row
    {
        bool edited;
        interval typed;
        typed_parameter shown;
    };

    struct asked_value
    {
        char name;
        double value;
    };

public:
    parameter_rows(scenario_reader seen, std::weak_ptr<owned_scenario> scenario);

    void render();

    std::vector<typed_parameter> held() const;

private:
    scenario_reader m_seen;
    std::vector<parameter_row> m_rows;
    std::optional<asked_value> m_wanted;
    std::optional<asked_value> m_in_flight;
    std::weak_ptr<owned_scenario> m_scenario;
    std::shared_ptr<const scenario_snapshot> m_last;

    void catch_up(const std::shared_ptr<const scenario_snapshot> &shown);
    void render_row(parameter_row &row);
    bool render_range(parameter_row &row);
    void post_wanted();
    bool waiting(char name) const;
};

// The field a function is typed into, labeled by its variables, with its reading under it, or where
// it does not read, the text marked where it failed and what was expected there. A text is asked of
// the scenario once the field is left after an edit or Enter is pressed, and only where it reads.
class typed_function_panel
{
public:
    explicit typed_function_panel(const scenario_window_inputs &inputs);

    void render();

    const std::string &asked() const;

    // The text last asked for and an instance per parameter shown; where that text reads, an
    // instance the document carries for a letter it does not use is taken out.
    std::vector<config::edit> settings_edits(const config::document &carried) const;

private:
    std::size_t m_dimension;
    std::string m_asked;
    parameter_rows m_rows;
    std::weak_ptr<owned_scenario> m_scenario;
    std::array<char, typed::expression_capacity + 1> m_text;

    void post(std::string text);
};

}

#endif
