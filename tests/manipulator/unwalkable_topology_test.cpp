#include "two_link_arm.h"

#include "praxis/manipulator/scene_robot_builder.h"
#include "praxis/manipulator/screw_chain_builder.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/model.h>

#include <vector>

using namespace praxis::fixture;

namespace {

// Well addressed and still not a tree: every entry names a link that exists, so the addressing
// corpus cannot carry these. The first two are refused for the chain they leave without an end, the
// last two for the root they give a parent.
std::vector<malformation> unwalkable_topologies()
{
    std::vector<malformation> corpus;
    corpus.push_back({"two links each named as the other's parent", two_link_arm("cyclic_parent_arm", meios::robot_topology{{1, 0}, {-1, 0}, {0}, {0, 1}})});
    corpus.push_back({"a link that is its own parent", two_link_arm("self_parent_arm", meios::robot_topology{{-1, 1}, {-1, 0}, {0}, {0, 1}})});
    corpus.push_back({"a link that is its own parent and is named a root", two_link_arm("self_parent_root_arm", meios::robot_topology{{-1, 1}, {-1, 0}, {0, 1}, {0, 1}})});
    corpus.push_back({"a root that has a parent", two_link_arm("parented_root_arm", meios::robot_topology{{-1, 0}, {-1, 0}, {1}, {0, 1}})});

    return corpus;
}

meios::model<> twice_rooted_arm()
{
    return two_link_arm("twice_rooted_arm", meios::robot_topology{{-1, 0}, {-1, 0}, {0, 0}, {0, 1}});
}

}

// The chain derivation refuses a cycle through its own walk and the scene graph walks one forever,
// so the two are asserted together: a model either comes back refused from both or the contract is
// broken however either one behaves alone.
TEST_CASE("both derivations refuse a topology whose parent table cannot be walked to a root")
{
    praxis::manipulator::screw_chain_options named;
    named.tip_link = "tool";

    for(const auto &[what, model] : unwalkable_topologies())
    {
        INFO(what);
        for(const auto &options : {praxis::manipulator::screw_chain_options(), named})
        {
            const auto chain = praxis::manipulator::build_screw_chain(model, options);

            REQUIRE_FALSE(chain.has_value());
            CHECK(chain.error() == praxis::refusal::degenerate);
        }

        const auto built = praxis::manipulator::build_scene_robot(model);

        REQUIRE_FALSE(built.has_value());
        CHECK(built.error() == praxis::refusal::degenerate);
    }
}

// A link named twice in the root table addresses a link every time, so the model is not ill formed
// and is not refused; only the emission may not read it twice.
TEST_CASE("a link named as a root twice contributes its subtree once")
{
    const auto built = praxis::manipulator::build_scene_robot(twice_rooted_arm());

    REQUIRE(built.has_value());
    CHECK((*built)->numDOF() == 1u);
}
