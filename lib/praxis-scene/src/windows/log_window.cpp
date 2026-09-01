#include "windows/log_window.h"

#include "praxis/scene/widgets.h"

#include "praxis/extension/held_handle.h"

#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>

#include <spdlog/sinks/base_sink.h>

#include <array>
#include <mutex>
#include <memory>
#include <string>
#include <cstddef>
#include <utility>

namespace praxis::scene {

namespace {

const std::array<const char *, 4> severity_labels{"debug", "info", "warning", "error"};

const std::array<ImVec4, 4> severity_colors{ImVec4(0.60f, 0.60f, 0.60f, 1.f), ImVec4(0.85f, 0.85f, 0.85f, 1.f), ImVec4(0.95f, 0.80f, 0.35f, 1.f), ImVec4(0.90f, 0.35f, 0.35f, 1.f)};

severity mapped(spdlog::level::level_enum level)
{
    if(level >= spdlog::level::err)
        return severity::error;
    if(level == spdlog::level::warn)
        return severity::warning;
    if(level == spdlog::level::info)
        return severity::info;

    return severity::debug;
}

// The pattern is the bare text with an empty line ending: a timestamped or level-prefixed line
// differs on every emission and would never coalesce, and a trailing newline would be drawn. The
// base's mutex is what guards the formatter, which caches; the ring guards itself, and the two are
// only ever taken in that order.
class buffer_sink : public spdlog::sinks::base_sink<std::mutex>
{
public:
    explicit buffer_sink(std::shared_ptr<log_buffer> messages)
            : base_sink(std::make_unique<spdlog::pattern_formatter>("%v", spdlog::pattern_time_type::local, std::string()))
            , m_messages(std::move(messages))
    {
        held(m_messages, "the log sink", "message buffer");
    }

protected:
    void sink_it_(const spdlog::details::log_msg &message) override
    {
        spdlog::memory_buf_t formatted;
        formatter_->format(message, formatted);
        m_messages->push(std::string(formatted.data(), formatted.size()), mapped(message.level));
    }

    void flush_() override
    {
    }

private:
    std::shared_ptr<log_buffer> m_messages;
};

}

std::size_t install_log_sink(std::shared_ptr<log_buffer> messages)
{
    held(messages, "the log sink", "message buffer");

    const std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
    logger->sinks().push_back(std::make_shared<buffer_sink>(std::move(messages)));

    return logger->sinks().size();
}

log_window::log_window(std::string name, std::shared_ptr<log_buffer> messages)
        : imgui_window(std::move(name))
        , m_level(reporting_level())
        , m_shown()
        , m_messages(std::move(messages))
{
    held(m_messages, "the message window", "message buffer");
}

void log_window::render()
{
    for(log_entry &entry : m_messages->drain())
        accept(std::move(entry));

    place_on_first_use();
    ImGui::Begin(display_name().c_str());
    render_level_control();
    for(const log_entry &entry : m_shown)
        render_entry(entry);
    if(ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.f);
    ImGui::End();
}

// The logger holds the level, so the control reads it back every frame rather than believing its own
// copy: anything else that moves the level is then visible here too.
void log_window::render_level_control()
{
    m_level = reporting_level();
    if(render_enum_selection("Reporting level", m_level, severity_labels))
        set_reporting_level(m_level);
}

// The ring coalesces only within one drain, so a message arriving once a frame reaches this as a
// fresh entry every frame and is coalesced again here against what is already displayed.
void log_window::accept(log_entry entry)
{
    if(!m_shown.empty() && same_message(m_shown.back(), entry))
    {
        m_shown.back().repeats += entry.repeats;
        return;
    }

    m_shown.push_back(std::move(entry));
    if(m_shown.size() > m_messages->capacity())
        m_shown.pop_front();
}

// The GUI library opens every window at one default corner, so a window that asks for no place of
// its own opens underneath the others. Both conditions yield to wherever the reader drags it.
void log_window::place_on_first_use() const
{
    const ImGuiViewport &viewport = *ImGui::GetMainViewport();
    const ImVec2 size(viewport.WorkSize.x * 0.45f, viewport.WorkSize.y * 0.25f);
    ImGui::SetNextWindowPos(ImVec2(viewport.WorkPos.x + 20.f, viewport.WorkPos.y + viewport.WorkSize.y - size.y - 20.f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
}

void log_window::render_entry(const log_entry &entry) const
{
    const std::size_t level = static_cast<std::size_t>(entry.level);
    ImGui::TextColored(severity_colors[level], "[%s]", severity_labels[level]);
    ImGui::SameLine();
    if(entry.repeats > 1)
        ImGui::Text("%s (x%u)", entry.text.c_str(), entry.repeats);
    else
        ImGui::TextUnformatted(entry.text.c_str());
}

}
