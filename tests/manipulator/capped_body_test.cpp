#include "robot/unit_mesh.h"
#include "robot/capped_body.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <algorithm>

using namespace praxis::manipulator;

namespace {

// The soup is written in single precision, so a coordinate read back off it agrees to about a part
// in ten million and no closer.
constexpr double read_back = 1.0e-6;

// How far off a plane a corner may stand and still count as lying on it, once its coordinates have
// been through a float and a facet's own normal has been dotted with them.
constexpr double on_a_plane = 1.0e-5;

const double out_of_reach = std::numeric_limits<double>::infinity();

const double ladder[]{0.05, 0.35, 0.75, 0.995, out_of_reach};

std::vector<Eigen::Vector3d> soup_of(const Eigen::Vector3d &half_widths)
{
    std::vector<float> into(3u * capped_body_vertex_bound());
    const std::size_t vertices = cap_unit_body(half_widths, into);

    std::vector<Eigen::Vector3d> read;
    for(std::size_t at = 0u; at < vertices; ++at)
        read.emplace_back(into[3u * at], into[3u * at + 1u], into[3u * at + 2u]);

    return read;
}

// How far past the mesh's own surface a point stands: zero on it, negative inside it.
double past_the_mesh(const Eigen::Vector3d &point)
{
    double most = -out_of_reach;
    for(const mesh_facet &facet : unit_mesh_facets())
        most = std::max(most, facet.out.dot(point) - facet.reach);

    return most;
}

bool on_a_face(const Eigen::Vector3d &point, const Eigen::Vector3d &half_widths, Eigen::Index other_than)
{
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        if(axis != other_than && std::abs(std::abs(point[axis]) - half_widths[axis]) < on_a_plane)
            return true;

    return false;
}

Eigen::Vector3d extent_of(const std::vector<Eigen::Vector3d> &read)
{
    Eigen::Vector3d widest = Eigen::Vector3d::Zero();
    for(const Eigen::Vector3d &point : read)
        widest = widest.cwiseMax(point.cwiseAbs());

    return widest;
}

}

TEST_CASE("The mesh is inscribed in the sphere it approximates", "[manipulator][drawing]")
{
    CHECK(unit_mesh_inradius() < 1.0);
    CHECK(unit_mesh_inradius() > 0.99);
    CHECK(unit_mesh_corners().size() == 3u * unit_mesh_facets().size());

    for(const Eigen::Vector3d &corner : unit_mesh_corners())
        REQUIRE(std::abs(corner.norm() - 1.0) < read_back);
}

TEST_CASE("A cut past every semi-axis leaves the mesh itself, corner for corner", "[manipulator][drawing]")
{
    const std::vector<Eigen::Vector3d> uncut = soup_of(Eigen::Vector3d::Constant(out_of_reach));
    const std::vector<Eigen::Vector3d> wide  = soup_of(Eigen::Vector3d::Constant(2.0));

    REQUIRE(uncut.size() == unit_mesh_corners().size());
    REQUIRE(wide.size() == uncut.size());
    for(std::size_t at = 0u; at < uncut.size(); ++at)
        REQUIRE((wide[at] - uncut[at]).cwiseAbs().maxCoeff() == 0.0);
}

TEST_CASE("A cut at the mesh's own inradius leaves that axis uncut and one just short of it cuts", "[manipulator][drawing]")
{
    const double reach = unit_mesh_inradius();

    CHECK(soup_of(Eigen::Vector3d(reach, out_of_reach, out_of_reach)).size() == unit_mesh_corners().size());

    const std::vector<Eigen::Vector3d> cut = soup_of(Eigen::Vector3d(0.99 * reach, out_of_reach, out_of_reach));
    CHECK(cut.size() != unit_mesh_corners().size());
    CHECK(std::abs(extent_of(cut).x() - 0.99 * reach) < read_back);
}

TEST_CASE("Every corner of a cut body lies inside its box, and an axis the cut misses keeps the mesh's reach", "[manipulator][drawing]")
{
    const Eigen::Vector3d half(0.5, 0.8, out_of_reach);
    const std::vector<Eigen::Vector3d> read = soup_of(half);

    REQUIRE_FALSE(read.empty());
    for(const Eigen::Vector3d &point : read)
        for(Eigen::Index axis = 0; axis < 2; ++axis)
            REQUIRE(std::abs(point[axis]) <= half[axis] + read_back);

    const Eigen::Vector3d widest = extent_of(read);
    CHECK(std::abs(widest.x() - half.x()) < read_back);
    CHECK(std::abs(widest.y() - half.y()) < read_back);
    CHECK(std::abs(widest.z() - 1.0) < read_back);
}

TEST_CASE("The curve where a face meets the surface is an edge of the soup", "[manipulator][drawing]")
{
    const Eigen::Vector3d half(0.5, 0.8, 0.9);
    const std::vector<Eigen::Vector3d> read = soup_of(half);

    for(Eigen::Index axis = 0; axis < 3; ++axis)
    {
        std::size_t edges = 0u;
        for(std::size_t at = 0u; at + 2u < read.size(); at += 3u)
            for(std::size_t corner = 0u; corner < 3u; ++corner)
            {
                const Eigen::Vector3d &from = read[at + corner];
                const Eigen::Vector3d &to   = read[at + (corner + 1u) % 3u];
                const bool along            = std::abs(std::abs(from[axis]) - half[axis]) < on_a_plane && std::abs(std::abs(to[axis]) - half[axis]) < on_a_plane;
                edges += static_cast<std::size_t>(along && from[axis] * to[axis] > 0.0 && (to - from).norm() > read_back);
            }

        CHECK(edges > 0u);
    }

    // A corner standing on a face was put there by a clip rather than pushed there, so it is still
    // on the surface it was cut out of, or else on another of the box's own planes.
    for(const Eigen::Vector3d &point : read)
        for(Eigen::Index axis = 0; axis < 3; ++axis)
            if(std::abs(std::abs(point[axis]) - half[axis]) < on_a_plane)
                REQUIRE((std::abs(past_the_mesh(point)) < on_a_plane || on_a_face(point, half, axis)));
}

TEST_CASE("A cut standing inside the mesh on every axis is the box and nothing else", "[manipulator][drawing]")
{
    const Eigen::Vector3d half(0.2, 0.3, 0.4);
    const std::vector<Eigen::Vector3d> read = soup_of(half);

    // Six rectangles, each fanned into two triangles, and no curved part left over.
    REQUIRE(read.size() == 36u);
    CHECK((extent_of(read) - half).cwiseAbs().maxCoeff() < read_back);

    for(std::size_t at = 0u; at + 2u < read.size(); at += 3u)
    {
        std::size_t flat = 0u;
        for(Eigen::Index axis = 0; axis < 3; ++axis)
        {
            const bool level = read[at][axis] == read[at + 1u][axis] && read[at][axis] == read[at + 2u][axis];
            flat += static_cast<std::size_t>(level && std::abs(std::abs(read[at][axis]) - half[axis]) < read_back);
        }

        REQUIRE(flat > 0u);
    }
}

TEST_CASE("A semi-axis the cut reaches on only one side is still cut on both", "[manipulator][drawing]")
{
    const std::vector<Eigen::Vector3d> read = soup_of(Eigen::Vector3d(out_of_reach, 0.4, out_of_reach));

    REQUIRE_FALSE(read.empty());
    CHECK(std::abs(extent_of(read).y() - 0.4) < read_back);

    std::size_t below = 0u;
    std::size_t above = 0u;
    for(const Eigen::Vector3d &point : read)
    {
        below += static_cast<std::size_t>(std::abs(point.y() + 0.4) < on_a_plane);
        above += static_cast<std::size_t>(std::abs(point.y() - 0.4) < on_a_plane);
    }

    CHECK(below > 0u);
    CHECK(above > 0u);
}

TEST_CASE("The corner count never passes the bound the unit states", "[manipulator][drawing]")
{
    for(const double first : ladder)
        for(const double second : ladder)
            for(const double third : ladder)
            {
                const std::vector<Eigen::Vector3d> read = soup_of(Eigen::Vector3d(first, second, third));
                REQUIRE(read.size() <= capped_body_vertex_bound());
                REQUIRE(read.size() % 3u == 0u);
            }
}
