#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_MESH_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_MESH_H

#include "praxis/optimization/scenario_snapshot.h"

#include <threepp/core/BufferGeometry.hpp>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <cstddef>

namespace praxis::optimization {

// Where a point of the surface stands in the scene's z-up frame: x and y are carried by one scale,
// so a right angle in the plane stays one, onto a square of side `surface_extent` about the origin
// centered on the ranges' middles, and z = vertical f, f = 0 standing on the floor.
struct field_placement
{
    Eigen::Vector2d middle;
    double across;
    double vertical;

    Eigen::Vector3d at(double x, double y, double f) const;
};

// The vertical scale stands the largest |f| over the grid's finite samples at `surface_height` of the
// extent; a grid with no finite sample other than zero is drawn flat.
field_placement placement_of(std::span<const interval> ranges, const scenario_grid &grid);

// Room for every vertex and every triangle of a grid of `count` samples per side, so the buffers the
// renderer holds are never replaced.
std::shared_ptr<threepp::BufferGeometry> field_geometry(std::size_t count);

// The vertices stand at the samples, the normals are the grid's central differences, one-sided at
// an edge or beside a sample that is not finite, and each cell is two triangles, a triangle with a
// corner at a sample that is not finite left out of the index, so the drawing has a gap there. The
// drawn range names the indices in use.
void write_field(threepp::BufferGeometry &shaped, const scenario_grid &grid, std::span<const interval> ranges, const field_placement &placed);

}

#endif
