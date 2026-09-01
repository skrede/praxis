#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_OPTION_WIDGETS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_OPTION_WIDGETS_H

#include "praxis/manipulator/option_cycle.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/axis_order.h"

#include <imgui.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <functional>

namespace praxis::manipulator {

constexpr const char *euler_input_labels[3]{"A", "B", "C"};

// A combo box over an option cycle's own labels. The cycle holds the selection across frames, which
// is what lets the combo preview the entry that is current rather than the first one.
template<typename E, std::uint8_t N>
bool render_option_cycle(const char *id, option_cycle<E, N> &options)
{
    int selected      = static_cast<int>(options.value_index().value_or(0u));
    const auto labels = options.labels();
    if(!ImGui::Combo(id, &selected, labels.data(), static_cast<int>(labels.size())))
        return false;

    return options.set(labels[static_cast<std::size_t>(selected)]);
}

inline void render_euler_inputs(const char *id, Eigen::Vector3f &euler_degrees, praxis::axis_order &order, float step, float step_fast,
                                const std::function<void(int idx)> &on_vector_change = nullptr, const std::function<void(praxis::axis_order)> &on_enum_change = nullptr)
{
    scene::render_float3_inputs(euler_degrees, euler_input_labels, step, step_fast, on_vector_change);
    if(scene::render_enum_selection(id, order, praxis::axis_order_labels()) && on_enum_change != nullptr)
        on_enum_change(order);
}

}

#endif
