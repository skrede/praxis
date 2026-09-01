#ifndef HPP_GUARD_PRAXIS_TESTS_SCENE_HELD_COMPOSITION_H
#define HPP_GUARD_PRAXIS_TESTS_SCENE_HELD_COMPOSITION_H

#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/scheduler.h"

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <memory>
#include <vector>
#include <cstddef>

namespace praxis::fixture {

using namespace scene;

inline std::size_t descendants(threepp::Object3D &root)
{
    std::size_t counted = 0;
    root.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

// One node in the scene for as long as the body is placed, which is what a case counts to decide
// whether a release happened.
class placed_body : public stencil
{
public:
    explicit placed_body(threepp::Scene &target)
            : m_target(target)
            , m_node(threepp::Object3D::create())
    {
    }

    expected<void, refusal> initialize() override
    {
        m_target.add(m_node);

        return {};
    }

    void tear_down() override
    {
        m_target.remove(*m_node);
    }

    void render() const override
    {
    }

private:
    threepp::Scene &m_target;
    std::shared_ptr<threepp::Object3D> m_node;
};

inline preset_registry::factory composing()
{
    return [](const preset_site &site)
    { return std::make_shared<preset>(std::make_shared<placed_body>(site.scene), std::vector<std::shared_ptr<imgui_window>>{}, site.add_window, site.remove_window); };
}

// What a case watches an installed question and its resolution through. `outstanding` is what the
// question answers, so a case decides whether there is anything left to decide.
struct decision_record
{
    bool outstanding;
    int asked;
    int resolved;
    leaving_answer given;
};

// One composition over a scene of its own, with the window routes every preset requires already
// installed. A question is installed only where a case asks for one.
struct held_scene
{
    scheduler::scheduler loop;
    decision_record watched;
    std::shared_ptr<threepp::Scene> target;
    composition held;

    explicit held_scene(bool outstanding)
            : loop(scheduler::inline_workers)
            , watched{outstanding, 0, 0, leaving_answer{}}
            , target(threepp::Scene::create())
            , held(*target, loop, {})
    {
        held.windows_through([](const std::shared_ptr<imgui_window> &) {}, [](const std::shared_ptr<imgui_window> &) {});
    }

    void asks()
    {
        held.asking_before_release(
                [this]
                {
                    ++watched.asked;

                    return watched.outstanding;
                },
                [this](leaving_answer chosen)
                {
                    ++watched.resolved;
                    watched.given = chosen;
                });
    }

    std::size_t counted() const
    {
        return descendants(*target);
    }
};

}

#endif
