#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_tree.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

constexpr double chain_tolerance = 1.0e-9;

transform turned(double radians, const Eigen::Vector3d &at)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = Eigen::AngleAxisd(radians, Eigen::Vector3d{1.0, -2.0, 3.0}.normalized()).toRotationMatrix();
    tf.block<3, 1>(0, 3) = at;

    return tf;
}

// Every placement carries a rotation of its own, so an answer that dropped one or wrote another over
// it differs from the answer that composed it.
const transform at_a = turned(0.4, Eigen::Vector3d{0.3, -0.2, 0.9});
const transform at_b = turned(-0.9, Eigen::Vector3d{1.1, 0.5, -0.4});
const transform at_c = turned(1.7, Eigen::Vector3d{-0.6, 0.8, 0.2});
const transform at_d = turned(0.55, Eigen::Vector3d{0.2, 1.3, -0.7});

// `above` is the stored index and `expressed_in` is the placement of the frame that index names. A
// removal is allowed to move the first and forbidden to change the second, so the two are read apart.
struct reading
{
    transform placement;
    transform world;
    transform expressed_in;
    std::optional<std::size_t> above;
};

reading read_one(const frame_tree &arranged, std::size_t index)
{
    const std::optional<std::size_t> above = arranged.parent_of(index);

    return reading{arranged.pose(index), arranged.world_pose(index), above ? arranged.pose(*above) : transform::Identity(), above};
}

std::vector<reading> read_every(const frame_tree &arranged)
{
    std::vector<reading> taken;
    for(std::size_t index = 0; index < arranged.count(); ++index)
        taken.push_back(read_one(arranged, index));

    return taken;
}

bool names_the_same_frames(const reading &put, const reading &was)
{
    return put.placement.isApprox(was.placement, chain_tolerance) && put.world.isApprox(was.world, chain_tolerance) && put.expressed_in.isApprox(was.expressed_in, chain_tolerance) &&
            put.above.has_value() == was.above.has_value();
}

bool reads_identically(const reading &put, const reading &was)
{
    return names_the_same_frames(put, was) && put.above == was.above;
}

frame_tree four_frames()
{
    frame_tree arranged(4, baseline().frame);
    arranged.set_pose(0, at_a);
    arranged.set_pose(1, at_b);
    arranged.set_pose(2, at_c);
    arranged.set_pose(3, at_d);

    return arranged;
}

}

TEST_CASE("a parent naming a frame after the removed one names that same frame afterwards")
{
    frame_tree arranged = four_frames();
    REQUIRE(arranged.set_parent(3, std::size_t{2}).has_value());

    REQUIRE(arranged.remove(1).has_value());

    REQUIRE(arranged.count() == 3);
    REQUIRE(arranged.parent_of(2) == std::optional<std::size_t>{1});
    REQUIRE(arranged.pose(1).isApprox(at_c));
    REQUIRE(arranged.pose(2).isApprox(at_d));
    REQUIRE(arranged.world_pose(2).isApprox(at_c * at_d, chain_tolerance));
}

TEST_CASE("a chain read across a removed position is the chain it was")
{
    frame_tree arranged = four_frames();
    REQUIRE(arranged.set_parent(2, std::size_t{1}).has_value());
    REQUIRE(arranged.set_parent(3, std::size_t{2}).has_value());

    const std::vector<reading> before = read_every(arranged);
    REQUIRE(arranged.remove(0).has_value());
    const std::vector<reading> after = read_every(arranged);

    REQUIRE(arranged.count() == 3);
    REQUIRE(after.size() + 1 == before.size());
    for(std::size_t index = 0; index < after.size(); ++index)
        REQUIRE(names_the_same_frames(after[index], before[index + 1]));

    REQUIRE(arranged.parent_of(0) == std::nullopt);
    REQUIRE(arranged.parent_of(1) == std::optional<std::size_t>{0});
    REQUIRE(arranged.parent_of(2) == std::optional<std::size_t>{1});
    REQUIRE(arranged.world_pose(2).isApprox(at_b * at_c * at_d, chain_tolerance));
}

TEST_CASE("a frame another frame's placement is expressed in is not removed")
{
    frame_tree arranged = four_frames();
    REQUIRE(arranged.set_parent(3, std::size_t{1}).has_value());

    const std::vector<reading> before     = read_every(arranged);
    const expected<void, refusal> refused = arranged.remove(1);
    const std::vector<reading> after      = read_every(arranged);

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == refusal::no_solution);
    REQUIRE(arranged.count() == before.size());
    for(std::size_t index = 0; index < after.size(); ++index)
        REQUIRE(reads_identically(after[index], before[index]));
}

TEST_CASE("an index past the end is refused by the removal and adds nothing")
{
    frame_tree arranged = four_frames();

    const std::vector<reading> before     = read_every(arranged);
    const expected<void, refusal> refused = arranged.remove(9);
    const std::vector<reading> after      = read_every(arranged);

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == refusal::unsupported_input);
    REQUIRE(arranged.count() == 4);
    for(std::size_t index = 0; index < after.size(); ++index)
        REQUIRE(reads_identically(after[index], before[index]));
}

TEST_CASE("an added frame takes the identity placement and moves no frame already there")
{
    frame_tree arranged = four_frames();
    REQUIRE(arranged.set_parent(3, std::size_t{2}).has_value());

    const std::vector<reading> before = read_every(arranged);
    const std::size_t joined          = arranged.add();
    const std::vector<reading> after  = read_every(arranged);

    REQUIRE(joined == 4);
    REQUIRE(arranged.count() == 5);
    REQUIRE(arranged.pose(joined).isApprox(transform::Identity()));
    REQUIRE(arranged.parent_of(joined) == std::nullopt);
    for(std::size_t index = 0; index < before.size(); ++index)
        REQUIRE(reads_identically(after[index], before[index]));
}

TEST_CASE("an add and the removal of what it added leave the tree as it was")
{
    frame_tree arranged = four_frames();
    REQUIRE(arranged.set_parent(2, std::size_t{1}).has_value());
    REQUIRE(arranged.set_parent(3, std::size_t{2}).has_value());

    const std::vector<reading> before = read_every(arranged);
    REQUIRE(arranged.remove(arranged.add()).has_value());
    const std::vector<reading> after = read_every(arranged);

    REQUIRE(arranged.count() == before.size());
    for(std::size_t index = 0; index < after.size(); ++index)
        REQUIRE(reads_identically(after[index], before[index]));
}

TEST_CASE("a tree emptied one frame at a time reads as an index past the end does")
{
    frame_tree arranged = four_frames();

    for(std::size_t taken = 0; taken < 4; ++taken)
        REQUIRE(arranged.remove(0).has_value());

    REQUIRE(arranged.count() == 0);
    REQUIRE(arranged.pose(0).isApprox(transform::Identity()));
    REQUIRE(arranged.world_pose(0).isApprox(transform::Identity()));
    REQUIRE(arranged.parent_of(0) == std::nullopt);
    REQUIRE(arranged.remove(0).error() == refusal::unsupported_input);
}
