#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/frame_window.h"
#include "praxis/rigid_motion/configuration.h"

#include <spdlog/spdlog.h>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <functional>
#include <string_view>

namespace praxis::rigid_motion {

namespace {

// Entry zero is the fixed frame the objects hang under, under the name the composition gave it; every
// later entry is one object of the stencil, in the stencil's own order.
std::vector<std::string> frame_entries(const frame_stencil &target)
{
    std::vector<std::string> entries{std::string(target.fixed_frame_name())};
    for(std::size_t index = 0; index < target.count(); ++index)
        entries.emplace_back(target.name_of(index));

    return entries;
}

}

frame_window::frame_window(std::string name, frame_stencil &target, const frame_ops &injected, const controls &offered, std::function<std::size_t()> selecting)
        : imgui_window(std::move(name))
        , m_frame(injected)
        , m_opened()
        , m_controls(offered)
        , m_selected(offered.first_panelled_object)
        , m_stencil(target)
        , m_refused()
        , m_settings_at()
        , m_objects(target.count())
        , m_entries(frame_entries(target))
        , m_selecting(std::move(selecting))
{
}

frame_window::frame_window(std::string name, frame_stencil &target, const frame_ops &injected, const settings &chosen, std::string at, const controls &offered,
                           std::function<std::size_t()> selecting)
        : imgui_window(std::move(name))
        , m_frame(injected)
        , m_opened(chosen)
        , m_controls(offered)
        , m_selected(offered.first_panelled_object)
        , m_stencil(target)
        , m_refused()
        , m_settings_at(std::move(at))
        , m_objects(chosen.objects)
        , m_entries(frame_entries(target))
        , m_selecting(std::move(selecting))
{
    m_objects.resize(target.count());
}

frame_window::settings frame_window::state() const
{
    return settings{m_objects};
}

std::vector<config::edit> frame_window::settings_edits(const config::document &carried) const
{
    const std::span<const std::string> objects = std::span(m_entries).subspan(1);

    return write_arrangement(carried, m_opened, state(), m_settings_at, objects);
}

// Asked only when the object set has changed size. A panel a person is typing into must not have its
// values read back out from under them, and the count is the only signal available without asking
// every object every frame.
void frame_window::resynchronize()
{
    std::vector<placement> rebuilt;
    rebuilt.reserve(m_stencil.count());
    for(std::size_t index = 0; index < m_stencil.count(); ++index)
        rebuilt.push_back(panel_of(index));

    m_entries = frame_entries(m_stencil);
    m_objects = std::move(rebuilt);
    if(!m_objects.empty() && m_selected >= m_objects.size())
        m_selected = m_objects.size() - 1;
}

// The conversion an assignment makes, run the other way: the pose the stencil carries for an object,
// read back as the position and the angles a panel shows at that panel's own axis order.
frame_window::placement frame_window::panel_of(std::size_t index) const
{
    const axis_order order        = index < m_objects.size() ? m_objects[index].order : placement().order;
    const transform &placed       = m_stencil.pose(index);
    const rotation held           = m_frame.rotation_matrix_from_transform(placed);
    const Eigen::Vector3d degrees = m_frame.euler_from_rotation_matrix(held, order) * degrees_per_radian;

    return placement{order, placed.block<3, 1>(0, 3).cast<float>(), degrees.cast<float>(), m_stencil.parent_of(index)};
}

void frame_window::initialize()
{
    for(std::size_t index = 0; index < m_objects.size(); ++index)
    {
        apply_parent(index, m_objects[index].parent);
        assign_pose(index);
    }
}

void frame_window::assign_pose(std::size_t index)
{
    const placement &driven      = m_objects[index];
    const Eigen::Vector3d angles = driven.euler_degrees.cast<double>() * radians_per_degree;
    const rotation orientation   = m_frame.rotation_matrix_from_euler(angles, driven.order);

    m_stencil.set_pose(index, m_frame.transformation_matrix_from_rotation_position(orientation, driven.position.cast<double>()));
}

void frame_window::apply_parent(std::size_t index, std::optional<std::size_t> chosen)
{
    m_refused.clear();
    if(m_stencil.set_parent(index, chosen))
    {
        m_objects[index].parent = chosen;

        return;
    }

    refuse_parent(index, chosen);
    m_objects[index].parent = m_stencil.parent_of(index);
}

// Shown in the panel beside the control that was pressed and kept there until the next attempt: a
// refusal that only reaches the log is a control that appears to do nothing.
void frame_window::refuse_parent(std::size_t index, std::optional<std::size_t> chosen)
{
    const std::string_view within = chosen ? m_stencil.name_of(*chosen) : std::string_view{};

    m_refused = "'" + std::string(m_stencil.name_of(index)) + "' cannot be expressed in '" + std::string(within) + "', so the arrangement is left as it was";
    spdlog::warn("praxis: {}", m_refused);
}

}
