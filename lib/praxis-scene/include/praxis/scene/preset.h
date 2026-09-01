#ifndef HPP_GUARD_PRAXIS_SCENE_PRESET_H
#define HPP_GUARD_PRAXIS_SCENE_PRESET_H

#include "praxis/scene/stencil.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/compat/detail/callable.h"
#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/task.h"
#include "praxis/scheduler/strand.h"

#include <memory>
#include <vector>

namespace praxis::scene {

// One loaded preset's lifetime. initialize() puts the body in the scene, then registers the windows,
// then admits the steppables; tear_down() withdraws the three in exactly the reverse order and frees
// nothing, because it runs inside the task that draws the frames and may not block.
class preset
{
public:
    preset(std::shared_ptr<scene::stencil> s, std::vector<std::shared_ptr<imgui_window>> w, window_route add, window_route remove);

    scheduler::strand work;
    std::shared_ptr<scene::stencil> stencil;
    std::vector<scheduler::task_handle> steppables;
    std::vector<std::shared_ptr<imgui_window>> windows;

    // Absent unless the composition put state on the strand above. It runs last and on that strand,
    // as the acknowledgment of its retirement, so nothing queued behind it can reach what it frees.
    detail::move_only_function<void()> release_cb;

    // Absent unless the composition that built this put one on it, and run off the strand above
    // rather than on it, at whichever of the three ends the strand's retirement reaches: serviced,
    // refused, or discarded with the loop's own end. Its argument is whether the acknowledgment above
    // ran, so it is run on every end and concludes on the ends where nothing else did.
    detail::move_only_function<void(bool)> release_fallback_cb;

    // The composer's own step of initialize(), absent unless it registers steppables. It answers with
    // the handles it admitted, which the preset then holds for as long as it lives.
    detail::move_only_function<expected<std::vector<scheduler::task_handle>, refusal>()> admit_cb;

    expected<void, refusal> initialize();
    void tear_down();

private:
    window_route m_add_window;
    window_route m_remove_window;

    expected<void, refusal> admit();

    void withdraw_windows();
};

}

#endif
