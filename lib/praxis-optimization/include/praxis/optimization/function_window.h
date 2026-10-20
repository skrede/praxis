#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_FUNCTION_WINDOW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_FUNCTION_WINDOW_H

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::optimization {

// Where the functions a scenario offers come from. The order is the label table's, which is what
// reading a label back as an index and casting relies on.
enum class function_source : std::uint8_t
{
    registered
};

std::span<const char *const> function_source_labels();

// Which registered function is drawn and solved, and the range it is drawn over, held here and asked
// of the scenario through its gate as they are edited. A range whose minimum does not lie below its
// maximum stays in the fields and is not asked for. Their keys sit at the root of the scenario's
// document.
class function_window : public scene::imgui_window, public config::configurable
{
public:
    function_window(std::string name, const scenario_window_inputs &inputs);

    void render() override;

    std::string_view settings_path() const override
    {
        return {};
    }

    std::vector<config::edit> settings_edits(const config::document &carried) const override;

    const config::configurable *as_configurable() const override
    {
        return this;
    }

private:
    function_source m_source;
    std::size_t m_selected;
    std::vector<interval> m_typed;
    std::vector<interval> m_posted;
    std::vector<std::string> m_registered;
    std::weak_ptr<owned_scenario> m_scenario;

    bool render_range(std::size_t coordinate);
    void post_range();
};

}

#endif
