#ifndef HPP_GUARD_PRAXIS_SCENE_COMPOSITION_H
#define HPP_GUARD_PRAXIS_SCENE_COMPOSITION_H

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/detail/callable.h"
#include "praxis/compat/expected.h"

#include "praxis/config/configurable.h"

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <vector>
#include <cstdint>
#include <filesystem>
#include <functional>

namespace praxis::scene {

// no_factory: the builder offered holds no target. no_strand: the scheduler had no strand left for
// the composition to own. refused: the builder answered with no composition.
enum class load_refusal : std::uint8_t
{
    no_factory,
    no_strand,
    refused
};

// What a person decides when they leave a composition whose values have moved: whether to keep
// them, and whether the same answer stands for every later leaving.
struct leaving_answer
{
    bool keep;
    bool remember;
};

// Whether anything is left to decide before what is held is released, and what carries the decision
// out. Both belong to the holder, because what a value is and where it would go is the holder's
// business and no composition's.
using leaving_question   = detail::move_only_function<bool()>;
using leaving_resolution = detail::move_only_function<void(leaving_answer)>;

// One composition over one scene: what it is built from, what it currently holds, and the moves
// between them. A load makes the strand the composition's own state belongs to and hands it to the
// builder; an unload retires that strand, so whatever the composition put there is freed by the
// acknowledgment and not where the unload was asked for. A load while one is held tears that one
// down first, so the two never coexist. Every move mutates the scene graph where it is called, so
// each is only ever reached from a handler on the strand the frames are drawn on.
class composition
{
public:
    composition(threepp::Scene &target, scheduler::scheduler &loop, std::filesystem::path root);

    ~composition();

    bool loaded() const;
    const std::shared_ptr<preset> &composed() const;

    // The windows the held composition registered that carry settings, in the order it registered
    // them, and none at all when nothing is held. The pointers are that composition's own windows,
    // reached only from the strand the frames are drawn on and valid no longer than it is.
    std::vector<const config::configurable *> configured() const;

    // The route an unload the held composition asks for itself is carried out by. Reconciling
    // whatever a holder shows of a preset is that holder's business, so the holder installs it;
    // without one such a request is recorded and nothing else.
    void unload_through(detail::move_only_function<void()> route);

    // The routes every preset this composition builds registers and withdraws its windows through.
    // The holder owns the window list, so the holder installs them, and a composition given neither
    // composes no preset that carries a window. A withdrawal runs at the release below, which the
    // destructor reaches, so whatever these routes touch outlives the composition.
    void windows_through(window_route add, window_route remove);

    // The question asked before what is held is released, and the route the answer is carried out
    // through. Both are asked on the task the frames are drawn on, which is where a release may
    // happen at all.
    void asking_before_release(leaving_question pending, leaving_resolution answered);

    // Neither answer is a default: a composition awaiting one stays held until an answer arrives.
    bool awaiting_answer() const;

    void answer(leaving_answer chosen);

    expected<void, load_refusal> load(const preset_registry::factory &builder);
    void unload();

    // Tears the held preset down and retires the strand it was given, which is where whatever it put
    // there is freed. That freeing is the retirement's acknowledgment wherever the loop services one,
    // and the preset's own fallback -- handed whether that acknowledgment ran -- in every case. A
    // composition holding nothing releases nothing, and the destructor calls this, so what is held
    // is released whether or not a holder asks.
    //
    // `concluded` runs behind that acknowledgment and on the preset's own strand when the
    // acknowledgment is serviced; where the retirement was asked for when it is refused or nothing
    // was held; and where the strand's queued work is discarded when the loop ends without servicing
    // it -- exactly once in every case. A caller ending the run carries the stop in it rather than
    // issuing one after this returns: a stop evacuates every strand's ready queue, and the
    // acknowledgment has just been admitted to one of them.
    void release(detail::move_only_function<void()> concluded = {});

private:
    bool m_awaiting;
    threepp::Scene &m_scene;
    window_route m_add_window;
    window_route m_remove_window;
    scheduler::scheduler &m_loop;
    std::filesystem::path m_root;
    std::shared_ptr<preset> m_preset;
    std::vector<std::weak_ptr<imgui_window>> m_configured;
    leaving_question m_asking_cb;
    leaving_resolution m_answered_cb;
    detail::move_only_function<void()> m_unload_cb;

    std::function<void()> unload_route(scheduler::strand own);

    // Unloads only when the composition currently held is the one the request came from, which is
    // what keeps a request issued by a composition already unloaded from unloading the next one.
    void unload_if(scheduler::strand_id from);

    // The routes a preset is handed, which record and forget the windows that carry settings before
    // the holder's own routes see them. A route the holder never installed is handed on uninstalled.
    window_route collecting_route();
    window_route forgetting_route();

    // Makes the strand the composition's own state belongs to, asks the builder over a site
    // carrying it, and retires that strand again if the builder answers with nothing.
    expected<std::shared_ptr<preset>, load_refusal> make_preset(const preset_registry::factory &builder);

    // Puts what the builder answered with in place, and releases it again through the path below if
    // its own initialization refuses.
    void place_held();

    // Ends what is held. The preset's fallback and `concluded` are settled together, at whichever of
    // the retirement's three ends is reached, and never where it is asked for.
    void retire_held(detail::move_only_function<void()> concluded);
};

}

#endif
