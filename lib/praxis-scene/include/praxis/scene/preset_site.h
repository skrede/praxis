#ifndef HPP_GUARD_PRAXIS_SCENE_PRESET_SITE_H
#define HPP_GUARD_PRAXIS_SCENE_PRESET_SITE_H

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/strand.h"

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <filesystem>
#include <functional>

namespace praxis::scene {

using window_route = std::function<void(const std::shared_ptr<imgui_window> &)>;

// What a preset is composed against: the scene it adds itself to, the strand its frames are drawn
// on, the strand its own state belongs to, the route it says it cannot continue through, the two
// routes its windows are registered and withdrawn through, and the root everything it writes is
// placed under. The unload route names the composition it was built for, so a request issued by a
// composition already unloaded reaches nothing. The window routes take no lock and are called only
// from the strand the frames are drawn on. An empty root leaves each writer's own choice of place in
// force.
struct preset_site
{
    threepp::Scene &scene;
    scheduler::strand render;
    scheduler::strand work;
    std::function<void()> ask_unload;
    window_route add_window;
    window_route remove_window;
    std::filesystem::path root;
};

}

#endif
