#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_FIGURES_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_FIGURES_H

#include "surface_mesh.h"

#include <threepp/objects/Mesh.hpp>

#include <threepp/core/Object3D.hpp>
#include <threepp/core/BufferGeometry.hpp>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <string>
#include <cstddef>
#include <cstdint>

namespace praxis::optimization {

// Lines and points are drawn over the field rather than depth-tested against it, so no relief hides
// them; a line is one pixel wide under any core profile, so its tone is all that carries it.
// `joined` draws one strip through the points, otherwise each pair of points is a segment.
std::shared_ptr<threepp::Object3D> overlay_lines(std::string name, std::uint32_t tone, std::size_t points, bool joined);
std::shared_ptr<threepp::Mesh> overlay_point(std::string name, std::uint32_t tone, std::shared_ptr<threepp::BufferGeometry> sphere);

// A convex polygon of up to `corners` corners, lit on both sides and depth-tested as the field is;
// `wired` draws its triangles' edges rather than filling them.
std::shared_ptr<threepp::Mesh> plane_patch(std::string name, std::uint32_t tone, std::size_t corners, bool wired);

// Each point (x, y, f) is placed as the field is and written into the buffers the object was made
// with, beyond whose room nothing is written; an object written nothing is hidden.
void write_points(threepp::Object3D &drawn, std::span<const Eigen::Vector3d> points, const field_placement &placed);
void write_polygon(threepp::Mesh &drawn, std::span<const Eigen::Vector3d> polygon, const field_placement &placed);
void place_point(threepp::Object3D &drawn, const Eigen::Vector3d &point, const field_placement &placed);

}

#endif
