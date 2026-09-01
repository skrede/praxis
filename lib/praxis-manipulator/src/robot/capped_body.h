#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CAPPED_BODY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CAPPED_BODY_H

#include <Eigen/Core>

#include <span>
#include <cstddef>

namespace praxis::manipulator {

// The most vertices cap_unit_body writes, three floats each, so a caller sizes its buffer at this
// once and names afterwards how much of it is in use. A triangle clipped against the six planes of
// a box keeps its own three corners and gains one where it crosses each plane, so it fans into at
// most seven triangles; each of the six flat faces is a polygon bounded by the four edges of its
// own rectangle and by at most one edge per triangle of the mesh.
std::size_t capped_body_vertex_bound();

// The triangle soup of the unit sphere's mesh intersected with the box of the three half-widths,
// three consecutive vertices per triangle, answering how many vertices were written. Every
// coordinate is in the domain where the ellipsoid is the unit sphere: a half-width is the cut
// divided by that axis's semi-axis, so a half-width of one is a cut standing exactly at the
// semi-axis, and a half-width at or above unit_mesh_inradius() leaves that axis uncut rather than
// cutting a sliver off the mesh's own corners. The surface is clipped against the planes rather
// than pushed onto them, so the curve where the two meet is an edge of the soup and no triangle
// spans it, and each flat face lies exactly in its own plane.
std::size_t cap_unit_body(const Eigen::Vector3d &half_widths, std::span<float> into);

}

#endif
