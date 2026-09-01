#ifndef HPP_GUARD_PRAXIS_SCENE_IMGUI_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_IMGUI_WINDOW_H

#include <threepp/canvas/Canvas.hpp>

#include <threepp/extras/imgui/ImguiContext.hpp>

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

struct ImPlotContext;

namespace praxis::config {
class configurable;
}

namespace praxis::scene {

// The GUI library identifies a panel by its title string, so the display name a window carries is
// that panel's identity and two windows given one name draw into one panel.
class imgui_window
{
public:
    explicit imgui_window(std::string name);
    imgui_window(const imgui_window &)            = delete;
    imgui_window(imgui_window &&)                 = delete;
    imgui_window &operator=(const imgui_window &) = delete;
    imgui_window &operator=(imgui_window &&)      = delete;
    virtual ~imgui_window()                       = default;

    const std::string &display_name() const;

    virtual void render() = 0;

    virtual void initialize()
    {
    }

    // A window carrying settings answers with itself, so what it stands for can be routed to the
    // document its preset owns; one carrying none answers nothing.
    virtual const config::configurable *as_configurable() const
    {
        return nullptr;
    }

private:
    std::string m_display_name;
};

// Where the GUI library's layout file goes under `root`: the root joined with the name that library
// writes the layout under, so a layout already written is found again in the new place. An empty root
// answers an empty path, which leaves the library's own choice of place in force.
std::filesystem::path layout_file(const std::filesystem::path &root);

// The GUI library borrows the path it is handed rather than copying it, so the string must outlive
// every read through that pointer. A placement holds the string, writes the layout out while the
// string is still alive, and unhooks the pointer before returning, so nothing reads it afterwards.
// An empty place hands the library nothing and leaves its own choice of place in force.
class placed_layout
{
public:
    explicit placed_layout(const std::filesystem::path &where);
    placed_layout(const placed_layout &)            = delete;
    placed_layout(placed_layout &&)                 = delete;
    placed_layout &operator=(const placed_layout &) = delete;
    placed_layout &operator=(placed_layout &&)      = delete;
    ~placed_layout();

    const std::string &where() const;

private:
    std::string m_where;
};

// The plotting library keeps a context of its own, reached through a single pointer as the GUI
// library's is. It must be created after the GUI context and destroyed before it.
class plot_context
{
public:
    plot_context();
    plot_context(const plot_context &)            = delete;
    plot_context(plot_context &&)                 = delete;
    plot_context &operator=(const plot_context &) = delete;
    plot_context &operator=(plot_context &&)      = delete;
    ~plot_context();

private:
    // The library makes a newly created context current only when none already is, so the standing
    // one has to be read before the new one is made: this declaration order is what orders the two.
    // It is cleared when the context it names is one this library made and that one goes away first.
    ImPlotContext *m_standing;
    ImPlotContext *m_made;
};

// The window list is edited and drawn only from handlers on the strand that owns the context, so it
// carries no lock: every route into it is a posted handler on that one strand.
class imgui_window_context : public ImguiFunctionalContext
{
public:
    imgui_window_context(const threepp::Canvas &canvas, const std::filesystem::path &root);
    imgui_window_context(const threepp::Canvas &canvas, const std::filesystem::path &root, std::vector<std::shared_ptr<imgui_window>> windows);

    void initialize();

    void add_window(std::shared_ptr<imgui_window> window);
    void remove_window(const std::shared_ptr<imgui_window> &window);

    void render_windows() const;

private:
    bool m_initialized;
    plot_context m_plots;
    placed_layout m_layout;
    std::vector<std::shared_ptr<imgui_window>> m_windows;
};

}

#endif
