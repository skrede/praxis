#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_UNIT_MESH_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_UNIT_MESH_H

#include <Eigen/Core>

#include <span>

namespace praxis::manipulator {

// One triangle of the mesh: the outward unit normal of its plane, the distance from the origin
// along that normal, and the box the triangle's own three corners span.
struct mesh_facet
{
    Eigen::Vector3d out;
    double reach;
    Eigen::Vector3d least;
    Eigen::Vector3d most;
};

// The triangle soup of the unit sphere's mesh, three consecutive corners per triangle, and one
// facet per triangle in the same order.
std::span<const Eigen::Vector3d> unit_mesh_corners();
std::span<const mesh_facet> unit_mesh_facets();

// The mesh is inscribed in the sphere it approximates, so its nearest facet plane stands this far
// from the origin rather than at one. A plane standing at or beyond it misses the mesh altogether.
double unit_mesh_inradius();

}

#endif
