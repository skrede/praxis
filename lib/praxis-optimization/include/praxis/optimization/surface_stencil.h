#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_STENCIL_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_STENCIL_H

#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/stencil.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/materials/Material.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <memory>
#include <vector>

namespace praxis::optimization {

// What a surface's graph draws, written by the window that switches it and read by the stencil, both
// on the strand the frames are drawn on.
struct surface_view
{
    bool field    = true;
    bool iterates = true;
};

// The surface a scenario publishes, drawn in the scene as a height field named "surface" with each
// iterate the run recorded as a sphere on it, under a node named "iterates" in the order recorded.
// The field is rebuilt only when a publication carries other samples, and the iterates are placed on
// every frame. An iterate outside the ranges, or where f was not answered, is not drawn, and the
// iterate the selected step starts from is drawn in a tone of its own. Everything is read from the
// publication, so nothing here evaluates the function, and nothing here picks: the seed is typed.
// The view says which of the two is drawn.
class surface_stencil : public scene::stencil
{
public:
    surface_stencil(threepp::Scene &parent, scenario_reader seen, std::shared_ptr<const surface_view> drawn);
    ~surface_stencil() override = default;

    expected<void, refusal> initialize() override;
    void tear_down() override;

    void render() const override;

private:
    threepp::Scene &m_scene;
    scenario_reader m_seen;
    std::shared_ptr<const surface_view> m_view;
    std::shared_ptr<threepp::Group> m_root;
    std::shared_ptr<threepp::Mesh> m_field;
    std::shared_ptr<threepp::Group> m_iterates;
    std::shared_ptr<threepp::Material> m_iterate_tone;
    std::shared_ptr<threepp::Material> m_selected_tone;
    std::shared_ptr<threepp::BufferGeometry> m_sphere;

    // Written while drawing, which the base class asks for on a const stencil.
    mutable std::shared_ptr<const scenario_grid> m_sampled;
    mutable std::vector<std::shared_ptr<threepp::Mesh>> m_markers;

    void place_iterates(const scenario_snapshot &shown) const;
};

}

#endif
