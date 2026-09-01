#ifndef HPP_GUARD_PRAXIS_SCENE_VISUALIZER_H
#define HPP_GUARD_PRAXIS_SCENE_VISUALIZER_H

#include "praxis/scene/log_buffer.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/detail/callable.h"

#include "praxis/config/configurable.h"

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/overrun.h"
#include "praxis/scheduler/scheduler.h"

#include <threepp/threepp.hpp>

#include <threepp/math/Color.hpp>

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <filesystem>

namespace praxis::scene {

// One admitted piece of stepped work as it stood when it was read. The tallies are copied rather
// than referred to, so a report outlives the composition it was taken from.
struct stepped_work_report
{
    bool valid;
    bool active;
    scheduler::task_counters counters;
};

// What an explicit write-back is asked for through. What is written and where it goes is the
// holder's business, so the holder installs it and the interface only asks.
using save_route = detail::move_only_function<void()>;

class visualizer
{
public:
    enum class projection
    {
        perspective,
        orthographic
    };

    // Where the operating-system window sits and how large it is, in the screen coordinates the
    // renderer reports. A member left absent leaves the renderer's own choice in place.
    struct geometry
    {
        std::optional<int> width;
        std::optional<int> height;
        std::optional<int> x;
        std::optional<int> y;
    };

    // What a visualizer is built with beyond its registry and the loop it draws on, the last of which
    // is the root everything written on its behalf is placed under; an empty root leaves each
    // writer's own choice of place in force.
    struct options
    {
        projection view;
        // `messages` lets a caller install the log sink before the visualizer exists, so what is
        // logged on the way here still reaches the window. Passing nothing makes a ring that starts
        // empty.
        std::shared_ptr<log_buffer> messages;
        geometry window;
        std::filesystem::path root;
    };

    visualizer(std::shared_ptr<preset_registry> registry, scheduler::scheduler &loop, options chosen = options());

    ~visualizer();

    void show_grid(bool show);
    void set_background_color(threepp::Color color);

    // Draws one frame and reports whether the application should keep going. The loop that calls it
    // belongs to the scheduler, and this is the strand every frame is drawn on.
    bool render_once();

    scheduler::strand executor() const;

    std::vector<std::string> preset_names() const;
    void load_preset(const std::string &name);
    void load_preset(const preset_registry::factory &preset_builder);
    void clear_preset();

    // Releases whatever is held, now and on the thread the frames are drawn on rather than through a
    // handler posted to it. It is what a holder asks for while the loop everything held depends on
    // is still running, so that the tear-down is acknowledged on the preset's own strand instead of
    // being concluded after a stop. Holding nothing releases nothing.
    //
    // `concluded` reaches the composition unchanged, so it runs behind that acknowledgment and
    // exactly once. A holder ending the run carries the stop of the loop in it.
    void release_preset(detail::move_only_function<void()> concluded = {});

    bool is_preset_loaded() const;

    // The question asked before a held composition is released, and the route the answer given is
    // carried out through. Both reach the composition unchanged.
    void asking_before_release(leaving_question pending, leaving_resolution answered);

    bool awaiting_answer() const;
    void answer(leaving_answer chosen);

    void saving_through(save_route route);

    bool save_offered() const;
    void save_values();

    // One entry per piece of stepped work the held composition admitted, in the order it holds
    // them, and none at all when nothing is held.
    std::vector<stepped_work_report> composed_work() const;

    // Which of the held composition's windows carry settings, in the order it registered them. What
    // is done with them is the holder's business and no composition's.
    std::vector<const config::configurable *> configured() const;

    // Where the window is now and how large it is, so a holder can persist it. What is persisted,
    // and where it goes, is the holder's business.
    geometry window_geometry() const;

private:
    save_route m_save_cb;
    projection m_projection;
    std::shared_ptr<log_buffer> m_messages;
    scheduler::strand m_strand;
    std::shared_ptr<preset_registry> m_reg;
    std::shared_ptr<threepp::Scene> m_scene;
    // Declared ahead of the canvas so that it is destroyed after one: the canvas keeps a raw
    // pointer to this, handed to it at construction.
    std::unique_ptr<threepp::IOCapture> m_capture;
    std::unique_ptr<threepp::Canvas> m_canvas;
    // Declared behind the canvas so that it is destroyed before one: it shuts its graphics backend
    // down against the canvas's window. Declared ahead of the composition so that it is destroyed
    // after one: a preset torn down at destruction withdraws its windows through this list.
    std::unique_ptr<imgui_window_context> m_imgui;
    composition m_composition;
    std::shared_ptr<threepp::Camera> m_camera;
    std::unique_ptr<threepp::GLRenderer> m_renderer;
    std::shared_ptr<threepp::GridHelper> m_grid_helper;
    std::shared_ptr<threepp::OrbitControls> m_controls;

    void unload();
    void load(const preset_registry::factory &builder, const std::string &name);

    void report_load_refusal(load_refusal reason, const std::string &name) const;

    void capture_input();
    void resize(threepp::WindowSize size);

    void setup_scene();
    void add_camera();
    void add_scene_grid();
    void add_scene_lights();
    void add_imgui_windows();
};

}

#endif
