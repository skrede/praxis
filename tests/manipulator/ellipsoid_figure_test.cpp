#include "robot/ellipsoid_figure.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/materials/MeshBasicMaterial.hpp>
#include <threepp/materials/LineBasicMaterial.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis::manipulator;

namespace {

// The vertices are held in single precision, so a coordinate read back off one agrees to about a
// part in ten million.
constexpr double read_back = 1.0e-6;

const Eigen::Vector3d flattened_axes{0.4, 0.2, 0.1};

std::shared_ptr<threepp::Object3D> a_body()
{
    return ellipsoid_object("body", threepp::MeshBasicMaterial::create());
}

std::vector<float> &vertices_of(const threepp::Object3D &drawn)
{
    return drawn.geometry()->getAttribute<float>("position")->array();
}

// The buffer is as wide as the widest shape there is, so the drawn range is what a body carries.
std::vector<Eigen::Vector3d> points_of(const threepp::Object3D &drawn)
{
    std::vector<Eigen::Vector3d> read;
    const std::vector<float> &held = vertices_of(drawn);
    const std::size_t held_now     = 3u * static_cast<std::size_t>(drawn.geometry()->drawRange.count);
    for(std::size_t at = 0; at + 2 < held_now && at + 2 < held.size(); at += 3)
        read.emplace_back(held[at], held[at + 1], held[at + 2]);

    return read;
}

Eigen::Vector3d extent_of(const threepp::Object3D &drawn)
{
    Eigen::Vector3d widest = Eigen::Vector3d::Zero();
    for(const Eigen::Vector3d &point : points_of(drawn))
        widest = widest.cwiseMax(point.cwiseAbs());

    return widest;
}

double apart(const Eigen::Vector3d &one, const Eigen::Vector3d &other)
{
    return (one - other).cwiseAbs().maxCoeff();
}

Eigen::Quaterniond turn_of(const threepp::Object3D &drawn)
{
    return Eigen::Quaterniond(drawn.quaternion.w, drawn.quaternion.x, drawn.quaternion.y, drawn.quaternion.z);
}

double on_the_ellipsoid(const Eigen::Vector3d &point, const Eigen::Vector3d &semi_axes)
{
    return point.cwiseQuotient(semi_axes).squaredNorm();
}

}

TEST_CASE("An uncapped body is the exact ellipsoid of its semi-axes", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = a_body();
    shape_ellipsoid(*drawn, flattened_axes, std::nullopt);

    CHECK(apart(extent_of(*drawn), flattened_axes) < read_back);

    for(const Eigen::Vector3d &point : points_of(*drawn))
        REQUIRE(std::abs(on_the_ellipsoid(point, flattened_axes) - 1.0) < read_back);
}

TEST_CASE("A cap cuts the body flat at the cap and nowhere widens it", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = a_body();
    shape_ellipsoid(*drawn, flattened_axes, 0.25);

    for(const Eigen::Vector3d &point : points_of(*drawn))
    {
        REQUIRE(point.cwiseAbs().maxCoeff() <= 0.25 + read_back);
        REQUIRE(on_the_ellipsoid(point, flattened_axes) <= 1.0 + read_back);
    }

    // The flat cut face carries the ellipsoid's own perpendicular width, so the two axes the cap
    // does not reach keep their extents.
    CHECK(apart(extent_of(*drawn), Eigen::Vector3d(0.25, 0.2, 0.1)) < read_back);

    // A cap standing under two of the semi-axes cuts both of them.
    shape_ellipsoid(*drawn, flattened_axes, 0.15);
    CHECK(apart(extent_of(*drawn), Eigen::Vector3d(0.15, 0.15, 0.1)) < read_back);
}

TEST_CASE("A semi-axis of zero draws a flat disc rather than nothing", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = a_body();
    shape_ellipsoid(*drawn, Eigen::Vector3d(0.4, 0.2, 0.0), std::nullopt);

    const std::vector<Eigen::Vector3d> read = points_of(*drawn);
    REQUIRE_FALSE(read.empty());
    for(const Eigen::Vector3d &point : read)
        REQUIRE(point.z() == 0.0);

    CHECK(apart(extent_of(*drawn), Eigen::Vector3d(0.4, 0.2, 0.0)) < read_back);

    // A cap divides by the semi-axes, so the axis that is gone is out of its reach.
    shape_ellipsoid(*drawn, Eigen::Vector3d(0.4, 0.2, 0.0), 0.15);
    CHECK(apart(extent_of(*drawn), Eigen::Vector3d(0.15, 0.15, 0.0)) < read_back);
}

TEST_CASE("Re-shaping a body allocates no buffer", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = a_body();
    const std::size_t opening                      = vertices_of(*drawn).size();
    const float *held                              = vertices_of(*drawn).data();

    shape_ellipsoid(*drawn, flattened_axes, std::nullopt);
    CHECK(vertices_of(*drawn).data() == held);

    shape_ellipsoid(*drawn, flattened_axes, 0.25);
    CHECK(vertices_of(*drawn).data() == held);
    CHECK(vertices_of(*drawn).size() == opening);
}

TEST_CASE("A body re-shaped to what it already carries is left alone", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = a_body();
    shape_ellipsoid(*drawn, flattened_axes, 0.25);

    // Written over by hand: a rewrite that ran puts the capped shape back, and one skipped does not.
    vertices_of(*drawn)[0] = 7.f;

    shape_ellipsoid(*drawn, flattened_axes, 0.25);
    CHECK(vertices_of(*drawn)[0] == 7.f);

    shape_ellipsoid(*drawn, flattened_axes, 0.2);
    CHECK(vertices_of(*drawn)[0] != 7.f);
}

TEST_CASE("A placed body carries its axes and its position and no scale", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = a_body();
    const Eigen::Matrix3d axes                     = Eigen::AngleAxisd(0.75, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
    const Eigen::Vector3d at{0.25, -0.5, 0.125};
    place_ellipsoid(*drawn, axes, at);

    CHECK(apart(Eigen::Vector3d(drawn->position.x, drawn->position.y, drawn->position.z), at) < read_back);
    CHECK(std::abs(std::abs(Eigen::Quaterniond(axes).dot(turn_of(*drawn))) - 1.0) < read_back);

    CHECK(drawn->scale.x == 1.f);
    CHECK(drawn->scale.y == 1.f);
    CHECK(drawn->scale.z == 1.f);
}

TEST_CASE("A continuation line is carried onto its axis by placement and scale", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = continuation_object("edge", threepp::LineBasicMaterial::create());

    const std::size_t opening = vertices_of(*drawn).size();
    const float *held         = vertices_of(*drawn).data();
    REQUIRE(opening == 6u);

    const Eigen::Vector3d from{0.5, 0.25, 0.125};
    place_continuation(*drawn, from, Eigen::Vector3d(2.0, 0.0, 0.0), 0.75);

    CHECK(vertices_of(*drawn).data() == held);
    CHECK(vertices_of(*drawn).size() == opening);
    CHECK(drawn->visible);
    CHECK(std::abs(static_cast<double>(drawn->scale.z) - 0.75) < read_back);
    CHECK(apart(Eigen::Vector3d(drawn->position.x, drawn->position.y, drawn->position.z), from) < read_back);

    // The line lives along its own +Z, so the placement is what has to turn it onto +X.
    CHECK(apart(turn_of(*drawn) * Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()) < read_back);
}

TEST_CASE("A continuation line of no length and one along no axis are not drawn", "[manipulator][drawing]")
{
    const std::shared_ptr<threepp::Object3D> drawn = continuation_object("edge", threepp::LineBasicMaterial::create());

    place_continuation(*drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.0);
    CHECK_FALSE(drawn->visible);

    place_continuation(*drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.75);
    CHECK_FALSE(drawn->visible);
    CHECK(std::isfinite(static_cast<float>(drawn->quaternion.w)));
}
