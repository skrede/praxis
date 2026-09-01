#ifndef HPP_GUARD_PRAXIS_SCENE_WIDGETS_H
#define HPP_GUARD_PRAXIS_SCENE_WIDGETS_H

#include <imgui.h>

#include <Eigen/Core>

#include <span>
#include <string>
#include <cstddef>
#include <functional>
#include <type_traits>

namespace praxis::scene {

inline void render_float3_inputs(Eigen::Vector3f &vector, const char *const labels[3], float step, float step_fast, const std::function<void(int idx)> &on_change = nullptr)
{
    if(ImGui::InputFloat(labels[0], &vector[0], step, step_fast) && on_change != nullptr)
        on_change(0);
    if(ImGui::InputFloat(labels[1], &vector[1], step, step_fast) && on_change != nullptr)
        on_change(1);
    if(ImGui::InputFloat(labels[2], &vector[2], step, step_fast) && on_change != nullptr)
        on_change(2);
}

inline void render_float_inputs_with_reset(float &f, const char *label, float step, float step_fast, const std::function<void()> &on_change = nullptr)
{
    if(ImGui::InputFloat(label, &f, step, step_fast) && on_change != nullptr)
        on_change();
    ImGui::SameLine();
    if(ImGui::Button((std::string("Reset ") + label).c_str()))
    {
        f = 0.f;
        if(on_change != nullptr)
            on_change();
    }
}

inline void render_float3_inputs_with_reset(Eigen::Vector3f &vector, const char *const labels[3], float step, float step_fast, const std::function<void(int idx)> &on_change = nullptr)
{
    render_float_inputs_with_reset(vector[0], labels[0], step, step_fast,
                                   [&]()
                                   {
                                       if(on_change != nullptr)
                                           on_change(0);
                                   });
    render_float_inputs_with_reset(vector[1], labels[1], step, step_fast,
                                   [&]()
                                   {
                                       if(on_change != nullptr)
                                           on_change(1);
                                   });
    render_float_inputs_with_reset(vector[2], labels[2], step, step_fast,
                                   [&]()
                                   {
                                       if(on_change != nullptr)
                                           on_change(2);
                                   });
}

inline void render_float_slider_with_reset(float &f, const char *label, float min, float max, const std::function<void()> &on_change = nullptr)
{
    if(ImGui::SliderFloat(label, &f, min, max) && on_change != nullptr)
        on_change();
    ImGui::SameLine();
    if(ImGui::Button((std::string("Reset ") + label).c_str()))
    {
        f = 0.f;
        if(on_change != nullptr)
            on_change();
    }
}

inline void render_float3_slider(Eigen::Vector3f &vector, const char *const labels[3], float min, float max, const std::function<void(int idx)> &on_change = nullptr)
{
    if(ImGui::SliderFloat(labels[0], &vector[0], min, max) && on_change != nullptr)
        on_change(0);
    if(ImGui::SliderFloat(labels[1], &vector[1], min, max) && on_change != nullptr)
        on_change(1);
    if(ImGui::SliderFloat(labels[2], &vector[2], min, max) && on_change != nullptr)
        on_change(2);
}

inline void render_float3_slider_with_reset(Eigen::Vector3f &vector, const char *const labels[3], float min, float max, const std::function<void(int idx)> &on_change = nullptr)
{
    render_float_slider_with_reset(vector[0], labels[0], min, max,
                                   [&]()
                                   {
                                       if(on_change != nullptr)
                                           on_change(0);
                                   });
    render_float_slider_with_reset(vector[1], labels[1], min, max,
                                   [&]()
                                   {
                                       if(on_change != nullptr)
                                           on_change(1);
                                   });
    render_float_slider_with_reset(vector[2], labels[2], min, max,
                                   [&]()
                                   {
                                       if(on_change != nullptr)
                                           on_change(2);
                                   });
}

// selected is in/out: a combo cannot preview the current entry unless the caller owns the
// selection across frames.
inline bool render_dropdown_selection(const char *id, std::size_t &selected, std::span<const std::string> entries)
{
    if(entries.empty() || !ImGui::BeginCombo(id, selected < entries.size() ? entries[selected].c_str() : ""))
        return false;

    bool changed = false;
    for(std::size_t i = 0; i < entries.size(); ++i)
        if(ImGui::Selectable(entries[i].c_str(), selected == i))
        {
            selected = i;
            changed  = true;
        }
    ImGui::EndCombo();
    return changed;
}

// Offers every entry of whatever enumeration it is asked about, which is what a caller naming no
// rule of its own asks for.
struct every_entry
{
    template<typename E>
    bool operator()(E) const
    {
        return true;
    }
};

// The labels are indexed by the enumerator's own value, which is what an enumeration numbered
// contiguously from zero buys; a sparse enumeration cannot be driven by this widget. Only the
// entries `offered` answers true of are drawn, so a value nothing can honor is not one anybody can
// pick; the entry the control stands on is previewed whether or not it is one of them.
template<typename E, typename offering = every_entry>
inline bool render_enum_selection(const char *id, E &value, std::span<const char *const> labels, offering offered = offering())
{
    const auto selected = static_cast<std::size_t>(static_cast<std::underlying_type_t<E>>(value));
    if(!ImGui::BeginCombo(id, selected < labels.size() ? labels[selected] : ""))
        return false;

    bool changed = false;
    for(std::size_t i = 0; i < labels.size(); ++i)
    {
        const auto entry = static_cast<E>(static_cast<std::underlying_type_t<E>>(i));
        if(offered(entry) && ImGui::Selectable(labels[i], selected == i))
        {
            value   = entry;
            changed = true;
        }
    }
    ImGui::EndCombo();
    return changed;
}

}

#endif
