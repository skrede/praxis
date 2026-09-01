#include "praxis/scene/preset.h"

#include "praxis/extension/held_handle.h"

#include <memory>
#include <vector>
#include <utility>
#include <stdexcept>

namespace praxis::scene {

preset::preset(std::shared_ptr<scene::stencil> s, std::vector<std::shared_ptr<imgui_window>> w, window_route add, window_route remove)
        : work()
        , stencil(std::move(s))
        , steppables()
        , windows(std::move(w))
        , release_cb()
        , release_fallback_cb()
        , admit_cb()
        , m_add_window(std::move(add))
        , m_remove_window(std::move(remove))
{
    held(stencil, "the preset", "body to render");
    for(const std::shared_ptr<imgui_window> &panel : windows)
        held(panel, "the preset", "window");
    if(m_add_window == nullptr || m_remove_window == nullptr)
        throw std::invalid_argument(detail::absent_handle("the preset", "window route"));
}

expected<void, refusal> preset::initialize()
{
    const expected<void, refusal> placed = stencil->initialize();
    if(!placed)
        return unexpected(placed.error());

    for(const std::shared_ptr<imgui_window> &panel : windows)
        m_add_window(panel);

    const expected<void, refusal> admitted = admit();
    if(!admitted)
    {
        withdraw_windows();
        stencil->tear_down();

        return unexpected(admitted.error());
    }

    return {};
}

// Destroying a handle is the non-blocking mark, and the state those tasks drove is freed by the
// strand retirement's acknowledgment rather than here.
void preset::tear_down()
{
    steppables.clear();
    withdraw_windows();
    stencil->tear_down();
}

expected<void, refusal> preset::admit()
{
    if(admit_cb == nullptr)
        return {};

    expected<std::vector<scheduler::task_handle>, refusal> admitted = admit_cb();
    if(!admitted)
        return unexpected(admitted.error());

    steppables = std::move(*admitted);

    return {};
}

void preset::withdraw_windows()
{
    for(auto panel = windows.rbegin(); panel != windows.rend(); ++panel)
        m_remove_window(*panel);
}

}
