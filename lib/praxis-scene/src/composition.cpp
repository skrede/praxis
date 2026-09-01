#include "praxis/scene/composition.h"

#include "praxis/compat/detail/callable.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <vector>
#include <utility>
#include <exception>
#include <functional>

namespace praxis::scene {

namespace {

// What a release still owes once the retirement has been asked for. Both steps run from the
// destructor and from nowhere else, so each runs exactly once wherever the last share is dropped, and
// the verdict the fallback is handed is whether the acknowledgment reached the mark. That destructor
// is reachable from pool::stop() and so from ~pool, which is itself a destructor: nothing may escape.
class settlement
{
public:
    settlement(detail::move_only_function<void(bool)> fallback, detail::move_only_function<void()> concluded)
            : m_marked(false)
            , m_fallback(std::move(fallback))
            , m_concluded(std::move(concluded))
    {
    }

    settlement(const settlement &)            = delete;
    settlement &operator=(const settlement &) = delete;

    ~settlement()
    {
        try
        {
            if(m_fallback != nullptr)
                m_fallback(!m_marked);
            if(m_concluded != nullptr)
                m_concluded();
        }
        catch(const std::exception &failed)
        {
            spdlog::error("praxis: the composition's release threw as it settled: {}", failed.what());
        }
        catch(...)
        {
            spdlog::error("praxis: the composition's release threw as it settled");
        }
    }

    void mark()
    {
        m_marked = true;
    }

private:
    bool m_marked;
    detail::move_only_function<void(bool)> m_fallback;
    detail::move_only_function<void()> m_concluded;
};

const char *refusal_name(refusal reason)
{
    switch(reason)
    {
        case refusal::unsupported_input:
            return "unsupported input";
        case refusal::degenerate:
            return "degenerate";
        case refusal::no_solution:
            return "no solution";
        case refusal::not_implemented:
            return "not implemented";
    }

    return "unclassified";
}

// The settlement is settled by the last share of it being dropped, so a retirement the scheduler
// refuses settles as this returns rather than on the strand.
void settle_strand(scheduler::scheduler &loop, preset &held, detail::move_only_function<void()> concluded)
{
    std::shared_ptr<settlement> settled = std::make_shared<settlement>(std::move(held.release_fallback_cb), std::move(concluded));
    static_cast<void>(loop.retire_strand(held.work,
                                         [settled, acknowledged = std::move(held.release_cb)]() mutable
                                         {
                                             settled->mark();
                                             if(acknowledged != nullptr)
                                                 acknowledged();
                                             // Dropping the last share here is what puts the settlement on this strand.
                                             settled.reset();
                                         }));
}

}

composition::composition(threepp::Scene &target, scheduler::scheduler &loop, std::filesystem::path root)
        : m_awaiting(false)
        , m_scene(target)
        , m_add_window()
        , m_remove_window()
        , m_loop(loop)
        , m_root(std::move(root))
        , m_preset()
        , m_configured()
        , m_asking_cb()
        , m_answered_cb()
        , m_unload_cb()
{
}

// The release runs a preset's tear-down, the holder's window withdrawals and the preset's own release
// callables, none of which this library authors, and a destructor is implicitly noexcept.
composition::~composition()
{
    try
    {
        release();
    }
    catch(const std::exception &failed)
    {
        spdlog::error("praxis: the composition's release threw at destruction: {}", failed.what());
    }
    catch(...)
    {
        spdlog::error("praxis: the composition's release threw at destruction");
    }
}

bool composition::loaded() const
{
    return m_preset != nullptr;
}

const std::shared_ptr<preset> &composition::composed() const
{
    return m_preset;
}

std::vector<const config::configurable *> composition::configured() const
{
    std::vector<const config::configurable *> shown;
    shown.reserve(m_configured.size());
    for(const std::weak_ptr<imgui_window> &held : m_configured)
    {
        const std::shared_ptr<imgui_window> panel = held.lock();
        const config::configurable *const carried = panel == nullptr ? nullptr : panel->as_configurable();
        if(carried != nullptr)
            shown.push_back(carried);
    }

    return shown;
}

void composition::unload_through(detail::move_only_function<void()> route)
{
    m_unload_cb = std::move(route);
}

void composition::windows_through(window_route add, window_route remove)
{
    m_add_window    = std::move(add);
    m_remove_window = std::move(remove);
}

void composition::asking_before_release(leaving_question pending, leaving_resolution answered)
{
    m_asking_cb   = std::move(pending);
    m_answered_cb = std::move(answered);
}

bool composition::awaiting_answer() const
{
    return m_awaiting;
}

void composition::answer(leaving_answer chosen)
{
    if(!m_awaiting)
        return;

    if(m_answered_cb != nullptr)
        m_answered_cb(chosen);

    m_awaiting = false;
    if(m_preset != nullptr)
        release();
}

// The request is carried on the strand the frames are drawn on, because that is where the scene
// graph may be mutated, and it names the strand it came from rather than being one-shot.
std::function<void()> composition::unload_route(scheduler::strand own)
{
    const scheduler::strand render      = m_loop.main_strand();
    const scheduler::strand_id identity = own.id();

    return [this, render, identity] { static_cast<void>(render.post([this, identity] { unload_if(identity); })); };
}

void composition::unload_if(scheduler::strand_id from)
{
    if(m_preset == nullptr || m_preset->work.id() != from || m_unload_cb == nullptr)
        return;

    m_unload_cb();
}

window_route composition::collecting_route()
{
    if(m_add_window == nullptr)
        return window_route{};

    return [this](const std::shared_ptr<imgui_window> &panel)
    {
        if(panel != nullptr && panel->as_configurable() != nullptr)
            m_configured.push_back(panel);
        m_add_window(panel);
    };
}

window_route composition::forgetting_route()
{
    if(m_remove_window == nullptr)
        return window_route{};

    return [this](const std::shared_ptr<imgui_window> &panel)
    {
        std::erase_if(m_configured, [&panel](const std::weak_ptr<imgui_window> &held) { return held.lock() == panel; });
        m_remove_window(panel);
    };
}

expected<std::shared_ptr<preset>, load_refusal> composition::make_preset(const preset_registry::factory &builder)
{
    const expected<scheduler::strand, scheduler::rejection> own = m_loop.make_strand();
    if(!own.has_value())
        return unexpected(load_refusal::no_strand);

    std::shared_ptr<preset> built = builder(preset_site{m_scene, m_loop.main_strand(), *own, unload_route(*own), collecting_route(), forgetting_route(), m_root});
    if(built == nullptr)
    {
        static_cast<void>(m_loop.retire_strand(*own, nullptr));
        return unexpected(load_refusal::refused);
    }

    built->work = *own;

    return built;
}

expected<void, load_refusal> composition::load(const preset_registry::factory &builder)
{
    if(builder == nullptr)
        return unexpected(load_refusal::no_factory);

    expected<std::shared_ptr<preset>, load_refusal> built = make_preset(builder);
    if(!built.has_value())
        return unexpected(built.error());

    // What is held is released only once the builder has answered, so a builder that answers
    // nothing leaves it exactly where it was.
    if(m_preset != nullptr)
        release();

    m_preset = std::move(*built);
    place_held();

    return {};
}

// The question is a gate in front of the release and no part of it: what the release does when it
// runs is what it has always done.
void composition::unload()
{
    if(m_preset == nullptr || m_awaiting)
        return;

    if(m_asking_cb != nullptr && m_asking_cb())
    {
        m_awaiting = true;
        return;
    }

    release();
}

void composition::place_held()
{
    const expected<void, refusal> placed = m_preset->initialize();
    if(placed)
        return;

    spdlog::error("praxis: the composition refused to initialize as {}, so it is unloaded again", refusal_name(placed.error()));
    release();
}

void composition::retire_held(detail::move_only_function<void()> concluded)
{
    // An answer is only ever awaited about what is held, so releasing what is held ends the wait.
    m_awaiting = false;
    // The callables are taken off the preset only once this has returned, so a tear-down that throws
    // leaves a preset that can still be released rather than one whose callables are gone.
    m_preset->tear_down();

    // A refused retirement settles inside this call, before what is held is dropped.
    settle_strand(m_loop, *m_preset, std::move(concluded));

    m_preset.reset();
    // A window registered through the route without being listed on the preset is withdrawn by
    // nothing, so the collection is emptied here rather than left to the withdrawals.
    m_configured.clear();
}

void composition::release(detail::move_only_function<void()> concluded)
{
    if(m_preset == nullptr)
    {
        if(concluded != nullptr)
            concluded();

        return;
    }

    retire_held(std::move(concluded));
}

}
