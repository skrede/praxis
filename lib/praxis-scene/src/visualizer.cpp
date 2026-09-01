#include "praxis/scene/visualizer.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/held_handle.h"

#include <GLFW/glfw3.h>

#include <spdlog/spdlog.h>

#include <string>
#include <memory>
#include <vector>
#include <utility>
#include <exception>
#include <unordered_map>

namespace praxis::scene {

namespace {

// A canvas is sized at construction and can only be placed once it owns a window, so the size the
// caller asked for is a construction parameter and the position is set on the window afterwards.
std::unique_ptr<threepp::Canvas> opened_canvas(const visualizer::geometry &window)
{
    std::unordered_map<std::string, threepp::Canvas::ParameterValue> params{{"aa", 4}, {"exitOnKeyEscape", false}};
    if(window.width && window.height)
        params.emplace("size", threepp::WindowSize{*window.width, *window.height});

    auto canvas = std::make_unique<threepp::Canvas>("praxis renderer", params);

    GLFWwindow *const handle = static_cast<GLFWwindow *>(canvas->windowPtr());
    if(handle && window.x && window.y)
        glfwSetWindowPos(handle, *window.x, *window.y);

    return canvas;
}

}

visualizer::visualizer(std::shared_ptr<preset_registry> registry, scheduler::scheduler &loop, options chosen)
        : m_save_cb()
        , m_projection(chosen.view)
        , m_messages(std::move(chosen.messages))
        , m_strand(loop.main_strand())
        , m_reg(std::move(registry))
        , m_scene(threepp::Scene::create())
        , m_composition(*m_scene, loop, chosen.root)
{
    held(m_reg, "the visualizer", "preset registry");
    m_composition.unload_through([this] { unload(); });

    m_canvas   = opened_canvas(chosen.window);
    m_renderer = std::make_unique<threepp::GLRenderer>(*m_canvas);
    m_renderer->setClearColor(threepp::Color::aliceblue);

    m_imgui = std::make_unique<imgui_window_context>(*m_canvas, chosen.root);

    m_composition.windows_through([this](const std::shared_ptr<imgui_window> &panel) { m_imgui->add_window(panel); },
                                  [this](const std::shared_ptr<imgui_window> &panel) { m_imgui->remove_window(panel); });

    setup_scene();
    capture_input();
}

// The held composition withdraws its windows through the interface context as it is released, so the
// release runs ahead of that context's own; the canvas is closed last, while the window the context
// shut its graphics backend down against is still alive. The context reset and the close therefore
// run whether or not the release threw, and a destructor is implicitly noexcept.
visualizer::~visualizer()
{
    try
    {
        m_composition.release();
    }
    catch(const std::exception &failed)
    {
        spdlog::error("praxis: the visualizer's release threw at destruction: {}", failed.what());
    }
    catch(...)
    {
        spdlog::error("praxis: the visualizer's release threw at destruction");
    }

    m_imgui.reset();

    if(m_canvas->isOpen())
        m_canvas->close();
}

void visualizer::show_grid(bool show)
{
    m_grid_helper->visible = show;
}

void visualizer::set_background_color(threepp::Color color)
{
    m_scene->background = color;
}

// The body writes the scene graph first, so the matrices updated and the frame drawn are the ones it
// just wrote rather than the ones the previous frame left.
bool visualizer::render_once()
{
    m_imgui->initialize();

    return m_canvas->animateOnce(
            [this]
            {
                if(m_composition.loaded())
                    m_composition.composed()->stencil->render();
                m_scene->updateMatrixWorld();
                m_renderer->render(*m_scene, *m_camera);
                m_imgui->render();
            });
}

scheduler::strand visualizer::executor() const
{
    return m_strand;
}

std::vector<std::string> visualizer::preset_names() const
{
    return m_reg->preset_names();
}

// A load or an unload is asked for from inside a rendered frame, so each gets a handler of its own
// on the strand the frames are drawn on rather than a flag the next frame reads. The admission
// verdict is discarded because a strand that admits nothing means the application is going away.
void visualizer::load_preset(const std::string &name)
{
    static_cast<void>(m_strand.post([this, name] { load(m_reg->load_preset(name), name); }));
}

void visualizer::load_preset(const preset_registry::factory &preset_builder)
{
    static_cast<void>(m_strand.post([this, preset_builder] { load(preset_builder, std::string()); }));
}

void visualizer::clear_preset()
{
    static_cast<void>(m_strand.post([this] { unload(); }));
}

void visualizer::release_preset(detail::move_only_function<void()> concluded)
{
    m_composition.release(std::move(concluded));
}

void visualizer::asking_before_release(leaving_question pending, leaving_resolution answered)
{
    m_composition.asking_before_release(std::move(pending), std::move(answered));
}

bool visualizer::awaiting_answer() const
{
    return m_composition.awaiting_answer();
}

// The answer arrives from inside a rendered frame and releases what is held, so it gets a handler of
// its own on the strand the frames are drawn on, as a load and an unload do.
void visualizer::answer(leaving_answer chosen)
{
    static_cast<void>(m_strand.post([this, chosen] { m_composition.answer(chosen); }));
}

void visualizer::saving_through(save_route route)
{
    m_save_cb = std::move(route);
}

bool visualizer::save_offered() const
{
    return m_save_cb != nullptr;
}

void visualizer::save_values()
{
    if(m_save_cb == nullptr)
        return;

    static_cast<void>(m_strand.post(
            [this]
            {
                if(m_save_cb != nullptr)
                    m_save_cb();
            }));
}

void visualizer::load(const preset_registry::factory &builder, const std::string &name)
{
    const expected<void, load_refusal> loaded = m_composition.load(builder);
    if(!loaded.has_value())
    {
        report_load_refusal(loaded.error(), name);
        return;
    }

    // A composition that answered can still have been released again by a refused initialization, so
    // what is announced is what the composition is holding rather than what it accepted. The overload
    // taking a factory carries no name and announces nothing.
    if(m_composition.loaded() && !name.empty())
        spdlog::info("Showing preset '{}'", name);
}

void visualizer::unload()
{
    m_composition.unload();
}

bool visualizer::is_preset_loaded() const
{
    return m_composition.loaded();
}

std::vector<stepped_work_report> visualizer::composed_work() const
{
    std::vector<stepped_work_report> reported;
    if(!m_composition.loaded())
        return reported;

    const std::vector<scheduler::task_handle> &admitted = m_composition.composed()->steppables;
    reported.reserve(admitted.size());
    for(const scheduler::task_handle &work : admitted)
        reported.push_back(stepped_work_report{work.valid(), work.active(), work.counters()});

    return reported;
}

std::vector<const config::configurable *> visualizer::configured() const
{
    return m_composition.configured();
}

visualizer::geometry visualizer::window_geometry() const
{
    const threepp::WindowSize measured = m_canvas->size();

    geometry where;
    where.width  = measured.width();
    where.height = measured.height();

    GLFWwindow *const handle = static_cast<GLFWwindow *>(m_canvas->windowPtr());
    if(!handle)
        return where;

    int left = 0;
    int top  = 0;
    glfwGetWindowPos(handle, &left, &top);
    where.x = left;
    where.y = top;

    return where;
}

void visualizer::report_load_refusal(load_refusal reason, const std::string &name) const
{
    if(reason == load_refusal::refused)
        spdlog::warn("Unable to load preset '{}': its composition was refused", name);
}

}
