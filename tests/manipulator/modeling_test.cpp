#include "praxis/manipulator/modeling.h"

#include "praxis/extension/coverage.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

expected<screw_chain, refusal> two_joint_chain(const meios::model<> &)
{
    return screw_chain(transform::Identity(), std::vector<screw_axis>(2, screw_axis::Zero()), joint_limits{});
}

}

TEST_CASE("the_modeling_slot_defaults_to_a_refusal_naming_the_case_it_does_not_implement")
{
    modeling_ops ops{};

    REQUIRE(ops.build_chain != nullptr);

    const auto chain = ops.build_chain(meios::model<>{});
    REQUIRE_FALSE(chain.has_value());
    REQUIRE(chain.error() == refusal::not_implemented);
}

TEST_CASE("assigning_the_modeling_slot_dispatches_to_the_assigned_function")
{
    modeling_ops ops{.build_chain = &two_joint_chain};

    REQUIRE(ops.build_chain == &two_joint_chain);

    const auto chain = ops.build_chain(meios::model<>{});
    REQUIRE(chain.has_value());
    REQUIRE(chain->joint_count() == 2);
}

TEST_CASE("the_modeling_slot_is_reported_as_carrying_the_refusal_channel")
{
    STATIC_REQUIRE(returns_refusal_v<decltype(modeling_ops{}.build_chain)>);
}
