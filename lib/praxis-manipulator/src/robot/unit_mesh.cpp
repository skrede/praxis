#include "robot/unit_mesh.h"

#include <threepp/core/BufferGeometry.hpp>

#include <threepp/geometries/SphereGeometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <span>
#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::manipulator {

namespace {

constexpr unsigned int width_segments  = 40;
constexpr unsigned int height_segments = 30;

// A triangle whose corners span less area than this has no plane of its own to speak of.
constexpr double degenerate = 1.0e-12;

struct built_mesh
{
    std::vector<Eigen::Vector3d> corners;
    std::vector<mesh_facet> facets;
    double inradius;
};

Eigen::Vector3d corner_at(const std::vector<float> &put, unsigned int which)
{
    const std::size_t at = 3u * static_cast<std::size_t>(which);

    return Eigen::Vector3d(put[at], put[at + 1u], put[at + 2u]);
}

mesh_facet facet_of(const std::array<Eigen::Vector3d, 3> &corners, const Eigen::Vector3d &across)
{
    const Eigen::Vector3d out = across.normalized();

    return mesh_facet{out, out.dot(corners[0]), corners[0].cwiseMin(corners[1]).cwiseMin(corners[2]), corners[0].cwiseMax(corners[1]).cwiseMax(corners[2])};
}

void add_triangle(built_mesh &built, const std::array<Eigen::Vector3d, 3> &corners)
{
    const Eigen::Vector3d across = (corners[1] - corners[0]).cross(corners[2] - corners[0]);
    if(across.norm() <= degenerate)
        return;

    const mesh_facet plane = facet_of(corners, across);
    built.corners.insert(built.corners.end(), corners.begin(), corners.end());
    built.facets.push_back(plane);
    built.inradius = std::min(built.inradius, plane.reach);
}

built_mesh grid()
{
    const std::shared_ptr<threepp::BufferGeometry> sphere = threepp::SphereGeometry::create(1.f, width_segments, height_segments);
    const std::vector<float> &put                         = sphere->getAttribute<float>("position")->array();
    const std::vector<unsigned int> &order                = sphere->getIndex()->array();

    built_mesh built;
    built.inradius = 1.0;
    for(std::size_t at = 0u; at + 2u < order.size(); at += 3u)
        add_triangle(built, {corner_at(put, order[at]), corner_at(put, order[at + 1u]), corner_at(put, order[at + 2u])});

    return built;
}

const built_mesh &mesh()
{
    static const built_mesh built = grid();

    return built;
}

}

std::span<const Eigen::Vector3d> unit_mesh_corners()
{
    return mesh().corners;
}

std::span<const mesh_facet> unit_mesh_facets()
{
    return mesh().facets;
}

double unit_mesh_inradius()
{
    return mesh().inradius;
}

}
