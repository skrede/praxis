#include "two_link_arm.h"

#include "captured_log.h"

#include "praxis/manipulator/scene_robot_builder.h"
#include "praxis/manipulator/screw_chain_builder.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <meios/model.h>

#include <string>
#include <vector>

using namespace praxis::tests;
using namespace praxis::fixture;

namespace {

std::vector<malformation> malformed_topologies()
{
    std::vector<malformation> corpus;
    corpus.push_back({"a traversal order naming a link past the list", two_link_arm("strayed_order_arm", meios::robot_topology{{-1, 0}, {-1, 0}, {0}, {0, 1, 7}})});
    corpus.push_back({"a traversal order carrying a negative index", two_link_arm("negative_order_arm", meios::robot_topology{{-1, 0}, {-1, 0}, {0}, {0, -1}})});
    corpus.push_back({"a root naming a link past the list", two_link_arm("strayed_root_arm", meios::robot_topology{{-1, 0}, {-1, 0}, {7}, {0, 1}})});
    corpus.push_back({"a parent naming a link past the list", two_link_arm("strayed_parent_arm", meios::robot_topology{{-1, 7}, {-1, 0}, {0}, {0, 1}})});
    corpus.push_back({"a parent negative without being the absence sentinel", two_link_arm("negative_parent_arm", meios::robot_topology{{-1, -2}, {-1, 0}, {0}, {0, 1}})});
    corpus.push_back({"a parent table shorter than the link list", two_link_arm("short_parent_arm", meios::robot_topology{{-1}, {-1, 0}, {0}, {0, 1}})});

    return corpus;
}

// The joint table is the one table the topology check leaves alone, because joint_above bounds both
// its index and its entry. Every reader of that table therefore has to go through joint_above.
std::vector<malformation> malformed_joint_tables()
{
    std::vector<malformation> corpus;
    corpus.push_back({"a joint table shorter than the link list", two_link_arm("short_joint_arm", meios::robot_topology{{-1, 0}, {-1}, {0}, {0, 1}})});
    corpus.push_back({"a joint entry naming a joint past the list", two_link_arm("strayed_joint_arm", meios::robot_topology{{-1, 0}, {-1, 7}, {0}, {0, 1}})});

    return corpus;
}

std::string diagnosis_of(const meios::model<> &model)
{
    captured_log captured;

    const auto derived = praxis::manipulator::build_screw_chain(model);

    return captured.text();
}

}

TEST_CASE("the arm the malformed fixtures are perturbed from derives")
{
    const auto derived = praxis::manipulator::build_screw_chain(well_formed_arm());

    REQUIRE(derived.has_value());
    REQUIRE(derived->joint_count() == 1);
}

// Both option forms, because they take different routes into the walk: an empty tip link selects one
// by searching, a named one looks it up directly, and only the second reaches the comparison that
// reads the root link's own name.
TEST_CASE("every derivation refuses a topology whose tables do not address the model's own links")
{
    praxis::manipulator::screw_chain_options named;
    named.tip_link = "tool";

    for(const auto &[what, model] : malformed_topologies())
    {
        INFO(what);
        for(const auto &options : {praxis::manipulator::screw_chain_options(), named})
        {
            const auto chain = praxis::manipulator::build_screw_chain(model, options);
            const auto names = praxis::manipulator::actuated_joint_names(model, options);
            const auto tip   = praxis::manipulator::tip_link_name(model, options);

            REQUIRE_FALSE(chain.has_value());
            REQUIRE_FALSE(names.has_value());
            REQUIRE_FALSE(tip.has_value());
            CHECK(chain.error() == praxis::refusal::degenerate);
            CHECK(names.error() == praxis::refusal::degenerate);
            CHECK(tip.error() == praxis::refusal::degenerate);
        }

        const auto built = praxis::manipulator::build_scene_robot(model);

        REQUIRE_FALSE(built.has_value());
        CHECK(built.error() == praxis::refusal::degenerate);
    }
}

TEST_CASE("the refusal names the table that does not address a link")
{
    for(const auto &[what, model] : malformed_topologies())
    {
        INFO(what);
        CHECK_THAT(diagnosis_of(model), Catch::Matchers::ContainsSubstring(model.name));
    }
}

TEST_CASE("the scene graph of the arm the malformed fixtures are perturbed from carries its joint")
{
    const auto built = praxis::manipulator::build_scene_robot(well_formed_arm());

    REQUIRE(built.has_value());
    REQUIRE((*built)->numDOF() == 1u);
}

// The scene graph reaches the joint table by a different route than the chain derivation does, and
// the table it reads is the one the topology check does not address.
TEST_CASE("a scene graph carries the joints the model's own table addresses and no others")
{
    for(const auto &[what, model] : malformed_joint_tables())
    {
        INFO(what);
        const auto built = praxis::manipulator::build_scene_robot(model);

        REQUIRE(built.has_value());
        CHECK((*built)->numDOF() == 0u);
    }
}
