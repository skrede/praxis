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

#include <threepp/core/Object3D.hpp>
#include <threepp/core/BufferGeometry.hpp>

#include <memory>
#include <vector>
#include <cstddef>

namespace praxis::optimization {

// What a surface's graph draws, written by the window that switches it and read by the stencil, both
// on the strand the frames are drawn on.
struct surface_view
{
    surface_view()
            : field(true)
            , iterates(true)
            , zero_curve(true)
            , step(true)
            , bounds(true)
            , point(true)
    {
    }

    bool field;
    bool iterates;
    bool zero_curve;
    bool step;
    bool bounds;
    bool point;
};

// The surface a scenario publishes, drawn in the scene as a height field named "surface" with each
// iterate the run recorded as a sphere on it, under a node named "iterates" in the order recorded,
// and its zero-level curve, "zero curve", on the zero plane. The field and the curve are rebuilt only
// when a publication carries other samples, and the iterates are placed on every frame. An iterate
// outside the ranges, or where f was not answered, is not drawn, and the iterate the selected step
// starts from is drawn in a tone of its own. Under "step" the selected step from x_k draws its
// "tangent plane", the plane's "zero line", the point of it nearest x_k, "nearest point", and the
// recorded x_(k+1) as "x_(k+1)", a line from the zero plane up to the surface. A problem carrying a
// box draws it, "box", on the zero plane. A point asked for by the Linearization window draws the
// tangent plane there, "tangent plane at p", over the ranges and held to the field's heights, and
// where the check disagrees the central difference's plane, "central difference at p", in wire.
// Everything is read from the publication, so nothing here evaluates the function, and nothing here
// picks: the seed is typed. The view says which of these is drawn.
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
    std::shared_ptr<threepp::Object3D> m_zero_curve;
    std::shared_ptr<threepp::Object3D> m_box;
    std::shared_ptr<threepp::Group> m_step;
    std::shared_ptr<threepp::Mesh> m_patch;
    std::shared_ptr<threepp::Object3D> m_zero_line;
    std::shared_ptr<threepp::Mesh> m_nearest;
    std::shared_ptr<threepp::Object3D> m_recorded;
    std::shared_ptr<threepp::Mesh> m_plane;
    std::shared_ptr<threepp::Mesh> m_central;

    // Written while drawing, which the base class asks for on a const stencil.
    mutable std::shared_ptr<const scenario_grid> m_sampled;
    mutable interval m_heights;
    mutable std::vector<std::shared_ptr<threepp::Mesh>> m_markers;

    void resample(const scenario_snapshot &shown) const;
    void place_iterates(const scenario_snapshot &shown) const;
    void hold_markers(std::size_t count) const;
};

}

#endif
