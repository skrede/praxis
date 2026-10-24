#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_LEVELS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_LEVELS_H

#include "surface_mesh.h"
#include "map_placement.h"
#include "surface_arrow.h"

#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/InstancedMesh.hpp>

#include <threepp/core/Object3D.hpp>
#include <threepp/core/BufferGeometry.hpp>

#include <threepp/materials/Material.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace praxis::optimization {

// The heights the map draws the level curves of: the k/(`level_count` + 1) quantiles of the grid's
// finite samples, k = 1..`level_count`, each interpolated linearly between the two sorted samples it
// falls between; none where no two finite samples differ.
std::vector<double> map_levels(const scenario_grid &grid);

// A minimized surface's level map: the level curves of f at `map_levels`, "level curves"; at the
// selected x_k an arrow along grad f(x_k), "gradient at x_k", horizontal and `gradient_arrow_fraction`
// of the extent long whatever |grad f|; and the step from x_k to the recorded x_(k+1), "step taken", a
// rod of `floor_step_radius`. On the field each curve stands at its own level, and the arrow and the
// rod, drawn over the field, at the centers of the field's markers. With the view's floor projection
// all three stand instead on a floor `floor_gap` of the extent below the field's lowest drawn point,
// beside the drawn iterates projected onto it, "iterates on the floor", the rod's axis
// `floor_step_lift` above it. Nothing is drawn from an x_k outside the ranges or whose gradient was not
// answered, and no step to an x_(k+1) that stands nowhere. The curves are traced again only when a
// publication carries other samples or the placement changes, and the rest is placed only when a
// publication is new or the placement changes, everything read from the publication. The map is hidden
// until `draw_map` shows it.
class level_map
{
public:
    explicit level_map(std::shared_ptr<threepp::BufferGeometry> sphere);

    const std::shared_ptr<threepp::Group> &object() const;

    void draw(const std::shared_ptr<const scenario_snapshot> &shown, const field_placement &placed, const scenario_view &view);
    void show(const scenario_view &view);

private:
    bool m_pointed;
    bool m_stepped;
    bool m_floored;
    double m_floor;
    surface_arrow m_arrow;
    std::shared_ptr<threepp::Group> m_map;
    std::shared_ptr<threepp::Mesh> m_step;
    std::shared_ptr<threepp::Object3D> m_curves;
    std::shared_ptr<threepp::Group> m_projected;
    std::shared_ptr<threepp::Material> m_mark_tone;
    std::shared_ptr<threepp::BufferGeometry> m_sphere;
    std::shared_ptr<threepp::InstancedMesh> m_marks;
    std::shared_ptr<const scenario_grid> m_sampled;
    std::shared_ptr<const scenario_snapshot> m_placed;

    void resample(const scenario_snapshot &shown, const field_placement &placed);
    void place(const scenario_snapshot &shown, const field_placement &placed);
    void step_from(const scenario_snapshot &shown, const map_placement &at);
    map_placement standing(const field_placement &placed) const;
};

// The materials whose opacity follows the run's form: the field's and the two linearization planes'.
struct form_materials
{
    threepp::Material &field;
    threepp::Material &plane;
    threepp::Material &central;
};

// The map over a run in the minimum form, with the field above it drawn at `minimum_field_opacity` and
// the planes at `minimum_plane_opacity`; no map, and the field and the planes opaque, over a run in the
// root form.
void draw_map(level_map &map, const form_materials &drawn, const std::shared_ptr<const scenario_snapshot> &shown, const field_placement &placed, const scenario_view &view);

}

#endif
