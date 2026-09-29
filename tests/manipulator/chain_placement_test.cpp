#include "robot/chain_placement.h"

#include "praxis/manipulator/screw_chain_builder.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/urdf/load.h>
#include <meios/model.h>

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>

// The descriptions are deployed beside the demonstration executable, so a configure without the
// demonstration leaves nothing to parse; that is a valid configuration and skips.
#ifndef PRAXIS_DEMO_RESOURCE_DIR
    #define PRAXIS_DEMO_RESOURCE_DIR ""
#endif

using namespace praxis;
using namespace praxis::manipulator;

namespace {

// Metres. The two deployed descriptions publish their dimensions to five decimals and write their
// right angles as truncated decimals, so a folded point agrees to about a nanometre and no closer.
constexpr double metre_scale = 1.0e-9;

constexpr const char *universal = "ur_description/urdf/ur.urdf.xacro";
constexpr const char *kuka      = "kuka_kr6_support/urdf/kr6r900sixx.xacro";

// The universal machine's description is one document parameterized over a whole product line, so
// which arm it describes is named by argument rather than by path.
std::optional<meios::model<>> deployed(const char *path, bool parameterized)
{
    const std::filesystem::path root{PRAXIS_DEMO_RESOURCE_DIR};
    if(root.empty() || !std::filesystem::exists(root / path))
        return std::nullopt;

    meios::load_options options;
    options.package_roots.push_back(root);
    options.eval       = meios::eval_policy::fail;
    options.on_missing = meios::missing_asset::fail;
    if(parameterized)
    {
        options.args["ur_type"] = "ur3e";
        options.args["name"]    = "ur3e";
    }

    auto loaded = meios::load(root / path, options);
    REQUIRE(loaded.has_value());

    return loaded->robot;
}

std::vector<Eigen::Vector3d> folded(const transform &home, std::span<const screw_axis> told, const joint_vector &theta)
{
    const expected<std::vector<Eigen::Vector3d>, refusal> answer = fold_joint_origins(home, told, theta, rigid_motion::baseline().screw);
    REQUIRE(answer.has_value());

    return *answer;
}

std::vector<Eigen::Vector3d> folded(const std::vector<screw_axis> &told)
{
    return folded(transform::Identity(), told, joint_vector::Zero(static_cast<Eigen::Index>(told.size())));
}

// Derived through the shipped builder, so that what is under test is the fold and not the
// derivation feeding it, at the configuration the published dimensions are quoted at.
std::vector<Eigen::Vector3d> folded_at_home(const meios::model<> &described)
{
    const expected<screw_chain, refusal> chain = build_screw_chain(described);
    REQUIRE(chain.has_value());

    return folded(chain->home, chain->space_screws, joint_vector::Zero(static_cast<Eigen::Index>(chain->joint_count())));
}

screw_axis revolute_about(const Eigen::Vector3d &through, const Eigen::Vector3d &along)
{
    return rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(through, along, 0.0).value();
}

}

// The seven lengths are the machine's own published shoulder height, upper arm, forearm and three
// wrist offsets, read off the description rather than off the fold.
TEST_CASE("a universal arm's chain stands its published link dimensions apart at the home configuration", "[manipulator][chain]")
{
    const std::optional<meios::model<>> described = deployed(universal, true);
    if(!described)
        SKIP("no robot description is deployed for this configuration");

    const std::vector<Eigen::Vector3d> points = folded_at_home(*described);
    REQUIRE(points.size() == 8u);

    const std::vector<double> published{0.0, 0.15185, 0.24355, 0.21320, 0.13105, 0.08535, 0.09210};
    for(std::size_t at = 0; at < published.size(); ++at)
        CHECK(std::abs((points[at + 1] - points[at]).norm() - published[at]) < metre_scale);
}

// The base structure the meshes carry below the first joint is in no screw, so the chain begins at
// the frame's origin and the pedestal is absent from it.
TEST_CASE("a kuka arm's chain stands at the joint origins its description publishes", "[manipulator][chain]")
{
    const std::optional<meios::model<>> described = deployed(kuka, false);
    if(!described)
        SKIP("no robot description is deployed for this configuration");

    const std::vector<Eigen::Vector3d> points = folded_at_home(*described);
    REQUIRE(points.size() == 8u);

    const std::vector<Eigen::Vector3d> published{{0.0, 0.0, 0.0},    {0.0, 0.0, 0.0},   {0.025, 0.0, 0.4}, {0.48, 0.0, 0.4},
                                                 {0.48, 0.0, 0.435}, {0.9, 0.0, 0.435}, {0.9, 0.0, 0.435}, {0.98, 0.0, 0.435}};

    for(std::size_t at = 0; at < published.size(); ++at)
        CHECK((points[at] - published[at]).norm() < metre_scale);
}

// A point-to-line projection is unique for every axis, which the foot of a common normal is not:
// parallel axes share no single common normal, and are what a construction built on one fails at.
TEST_CASE("three mutually parallel consecutive axes fold without a degenerate case", "[manipulator][chain]")
{
    const std::vector<screw_axis> told{revolute_about(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()), revolute_about(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()),
                                       revolute_about(Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d::UnitZ())};

    const std::vector<Eigen::Vector3d> points = folded(told);
    REQUIRE(points.size() == 5u);

    CHECK((points[1] - Eigen::Vector3d(0.0, 0.0, 0.0)).norm() < metre_scale);
    CHECK((points[2] - Eigen::Vector3d(1.0, 0.0, 0.0)).norm() < metre_scale);
    CHECK((points[3] - Eigen::Vector3d(2.0, 0.0, 0.0)).norm() < metre_scale);
    for(const Eigen::Vector3d &at : points)
        CHECK(at.allFinite());
}

TEST_CASE("a screw with no angular part names no axis to project onto and carries the point before it forward", "[manipulator][chain]")
{
    screw_axis sliding;
    sliding << Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ();

    const std::vector<screw_axis> told{revolute_about(Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3d::UnitZ()), sliding,
                                       revolute_about(Eigen::Vector3d(0.3, 0.7, 0.0), Eigen::Vector3d::UnitZ())};

    const std::vector<Eigen::Vector3d> points = folded(told);
    REQUIRE(points.size() == 5u);

    CHECK((points[1] - Eigen::Vector3d(0.3, 0.0, 0.0)).norm() < metre_scale);
    CHECK((points[2] - points[1]).norm() < metre_scale);
    CHECK((points[3] - Eigen::Vector3d(0.3, 0.7, 0.0)).norm() < metre_scale);
}

// The state an arm nobody has modelled yet opens at. Every point collapsing onto the origin is a
// figure no placement taken from the rendered arm could produce.
TEST_CASE("every screw the unit z axis through the origin collapses the whole chain onto the origin", "[manipulator][chain]")
{
    const std::vector<Eigen::Vector3d> points = folded(std::vector<screw_axis>(6u, revolute_about(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ())));
    REQUIRE(points.size() == 8u);

    for(const Eigen::Vector3d &at : points)
        CHECK(at.norm() < metre_scale);
}

// A pair of consecutive axes that meet folds to one point twice, and the polyline carries the
// segment of zero length rather than dropping it: its length is the joint count and two, whatever
// configuration it is folded at.
TEST_CASE("the polyline carries one point per joint beside the origin and the tool point at every configuration", "[manipulator][chain]")
{
    const std::vector<screw_axis> told{revolute_about(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()), revolute_about(Eigen::Vector3d(0.4, 0.0, 0.0), Eigen::Vector3d::UnitY()),
                                       revolute_about(Eigen::Vector3d(0.4, 0.0, 0.0), Eigen::Vector3d::UnitX())};

    const std::vector<Eigen::Vector3d> points = folded(told);
    REQUIRE(points.size() == told.size() + 2u);
    CHECK((points[3] - points[2]).norm() < metre_scale);

    joint_vector turned(3);
    turned << 0.3, -0.6, 1.1;

    CHECK(folded(transform::Identity(), told, turned).size() == told.size() + 2u);
}
