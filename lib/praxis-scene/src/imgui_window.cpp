#include "praxis/scene/imgui_window.h"

#include "praxis/extension/held_handle.h"

#include <imgui.h>
#include <implot.h>

#include <cmath>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <filesystem>

namespace praxis::scene {

namespace {

float crisp_font_scale(float content_scale)
{
    return std::max(1.f, std::round(content_scale));
}

// A plotting context this library made, beside the slot naming what it displaced. Whoever holds two
// of them chooses the order they go in, so a context being destroyed clears every slot still naming
// it before the plotting library frees it and those names are left dangling.
struct standing_context
{
    ImPlotContext *made;
    ImPlotContext **displaced;
};

std::vector<standing_context> &standing_contexts()
{
    static std::vector<standing_context> standing;

    return standing;
}

void enroll_context(ImPlotContext *made, ImPlotContext **displaced)
{
    standing_contexts().push_back({made, displaced});
}

void retire_context(ImPlotContext *made)
{
    std::vector<standing_context> &standing = standing_contexts();

    std::erase_if(standing, [made](const standing_context &one) { return one.made == made; });
    for(const standing_context &one : standing)
        if(*one.displaced == made)
            *one.displaced = nullptr;
}

}

imgui_window::imgui_window(std::string name)
        : m_display_name(std::move(name))
{
}

const std::string &imgui_window::display_name() const
{
    return m_display_name;
}

std::filesystem::path layout_file(const std::filesystem::path &root)
{
    return root.empty() ? std::filesystem::path() : root / "imgui.ini";
}

placed_layout::placed_layout(const std::filesystem::path &where)
        : m_where(where.string())
{
    if(m_where.empty())
        return;

    ImGui::GetIO().IniFilename = m_where.c_str();
}

placed_layout::~placed_layout()
{
    if(m_where.empty() || ImGui::GetCurrentContext() == nullptr)
        return;

    ImGuiIO &io = ImGui::GetIO();
    if(io.IniFilename != m_where.c_str())
        return;

    // The library loads a layout on the first frame and holds none before that, so a placement no
    // frame reached would write an empty file over a layout already on disk.
    if(ImGui::GetFrameCount() > 0)
        ImGui::SaveIniSettingsToDisk(m_where.c_str());
    io.IniFilename = nullptr;
}

const std::string &placed_layout::where() const
{
    return m_where;
}

plot_context::plot_context()
        : m_standing(ImPlot::GetCurrentContext())
        , m_made(ImPlot::CreateContext())
{
    enroll_context(m_made, &m_standing);
    ImPlot::SetCurrentContext(m_made);
}

plot_context::~plot_context()
{
    retire_context(m_made);

    // What was displaced is put back only while this object is still the one the library names, and
    // destroying the current context clears that name, so the two have to be compared beforehand.
    const bool naming_this = ImPlot::GetCurrentContext() == m_made;

    ImPlot::DestroyContext(m_made);
    if(naming_this)
        ImPlot::SetCurrentContext(m_standing);
}

imgui_window_context::imgui_window_context(const threepp::Canvas &canvas, const std::filesystem::path &root)
        : ImguiFunctionalContext(canvas.windowPtr(), [this] { render_windows(); })
        , m_initialized(false)
        , m_plots()
        , m_layout(layout_file(root))
{
}

imgui_window_context::imgui_window_context(const threepp::Canvas &canvas, const std::filesystem::path &root, std::vector<std::shared_ptr<imgui_window>> windows)
        : ImguiFunctionalContext(canvas.windowPtr(), [this] { render_windows(); })
        , m_initialized(false)
        , m_plots()
        , m_layout(layout_file(root))
        , m_windows(std::move(windows))
{
    for(const std::shared_ptr<imgui_window> &panel : m_windows)
        held(panel, "the window context", "window");
}

void imgui_window_context::initialize()
{
    if(m_initialized)
        return;

#ifdef __APPLE__
    // GLFW reports window size in points here and the ImGui GLFW backend already
    // applies the backing-scale factor; threepp's content-scale would double it.
    setFontScale(1.f);
#else
    setFontScale(crisp_font_scale(threepp::monitor::contentScale().first));
#endif
    for(auto &item : m_windows)
        item->initialize();
    m_initialized = true;
}

void imgui_window_context::add_window(std::shared_ptr<imgui_window> window)
{
    if(window == nullptr)
        return;
    m_windows.push_back(std::move(window));
    if(m_initialized)
        m_windows.back()->initialize();
}

void imgui_window_context::remove_window(const std::shared_ptr<imgui_window> &window)
{
    std::erase(m_windows, window);
}

void imgui_window_context::render_windows() const
{
    for(auto &item : m_windows)
        item->render();
}

}
