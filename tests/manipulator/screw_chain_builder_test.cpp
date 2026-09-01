#include "captured_log.h"

#include "praxis/manipulator/screw_chain_builder.h"
#include "praxis/manipulator/conversions.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <meios/model.h>

#include <string>
#include <vector>
#include <memory>
#include <numbers>
#include <utility>
#include <optional>

namespace {

using limits = meios::joint_limits<double>;

meios::link<> link(std::string name)
{
    meios::link<> record{};
    record.name = std::move(name);

    return record;
}

meios::joint<> joint(std::string name, std::string parent, std::string child, meios::joint_kind kind)
{
    meios::joint<> record{};
    record.name   = std::move(name);
    record.kind   = kind;
    record.parent = std::move(parent);
    record.child  = std::move(child);
    record.axis   = {1.0, 0.0, 0.0};

    return record;
}

// A shoulder rotating about its own x, quarter-turned about the root z, then a slide along the same
// local x, then a fixed tool offset. The quarter turn is what makes the derivation's rotation of a
// joint axis into the root frame observable: without it both screws would still come out unit-length
// and the home pose would still be a rigid motion.
meios::model<> quarter_turned_arm(std::optional<limits> shoulder_limits, meios::joint_kind shoulder_kind)
{
    meios::model<> model;
    model.name  = "quarter_turned_arm";
    model.links = {link("base"), link("shoulder"), link("slide"), link("tool")};

    auto shoulder            = joint("shoulder", "base", "shoulder", shoulder_kind);
    shoulder.origin          = {{0.0, 0.0, 0.3}, {0.0, 0.0, std::numbers::pi / 2.0}};
    shoulder.limits          = shoulder_limits;
    auto slide               = joint("slide", "shoulder", "slide", meios::joint_kind::prismatic);
    slide.origin.translation = {0.5, 0.0, 0.0};
    slide.limits             = limits{0.0, 0.4, 0.0, 0.5};
    auto tool                = joint("tool", "slide", "tool", meios::joint_kind::fixed);
    tool.origin.translation  = {0.0, 0.0, 0.1};
    model.joints             = {shoulder, slide, tool};

    meios::log_sink silent;
    model.topo = meios::reconstruct_topology(model.links, model.joints, silent, meios::topology_policy::skip).topo;

    return model;
}

meios::model<> bounded_arm()
{
    return quarter_turned_arm(limits{-1.5, 1.5, 0.0, 2.0}, meios::joint_kind::revolute);
}

meios::model<> zero_axis_arm()
{
    meios::model<> model = bounded_arm();
    model.joints[0].axis = {0.0, 0.0, 0.0};

    return model;
}

meios::model<> fixed_only_arm()
{
    meios::model<> model;
    model.name   = "fixed_only_arm";
    model.links  = {link("base"), link("tool")};
    model.joints = {joint("tool", "base", "tool", meios::joint_kind::fixed)};

    meios::log_sink silent;
    model.topo = meios::reconstruct_topology(model.links, model.joints, silent, meios::topology_policy::skip).topo;

    return model;
}

meios::model<> reconstructed(meios::model<> model)
{
    meios::log_sink silent;
    model.topo = meios::reconstruct_topology(model.links, model.joints, silent, meios::topology_policy::skip).topo;

    return model;
}

meios::model<> link_less_arm()
{
    meios::model<> model;
    model.name = "link_less_arm";

    return model;
}

// Every link is the child of some joint, so the reconstruction finds no root at all.
meios::model<> rootless_arm()
{
    meios::model<> model;
    model.name   = "rootless_arm";
    model.links  = {link("base"), link("shoulder")};
    model.joints = {joint("down", "base", "shoulder", meios::joint_kind::revolute), joint("up", "shoulder", "base", meios::joint_kind::revolute)};

    return reconstructed(std::move(model));
}

// A second root carries a branch of its own, which the first root's chain never reaches.
meios::model<> detached_arm()
{
    meios::model<> model;
    model.name   = "detached_arm";
    model.links  = {link("base"), link("carriage"), link("hand")};
    model.joints = {joint("lift", "carriage", "hand", meios::joint_kind::revolute)};

    return reconstructed(std::move(model));
}

// The two links below the root are each other's parent, so walking up from either never terminates.
meios::model<> cyclic_branch_arm()
{
    meios::model<> model;
    model.name   = "cyclic_branch_arm";
    model.links  = {link("base"), link("upper"), link("lower")};
    model.joints = {joint("down", "upper", "lower", meios::joint_kind::revolute), joint("up", "lower", "upper", meios::joint_kind::revolute)};

    return reconstructed(std::move(model));
}

// A topology assembled outside the reconstruction: it names a root that no link's parent chain
// reaches, which the reconstruction itself cannot produce.
meios::model<> unreachable_root_arm()
{
    meios::model<> model;
    model.name  = "unreachable_root_arm";
    model.links = {link("base"), link("hand")};
    model.topo  = meios::robot_topology{{-1, -1}, {-1, -1}, {1}, {0}};

    return model;
}

// A joint table one entry shorter than the link list, so the deepest link's lookup falls past it.
meios::model<> short_joint_table_arm()
{
    meios::model<> model;
    model.name   = "short_joint_table_arm";
    model.links  = {link("base"), link("shoulder"), link("wrist")};
    model.joints = {joint("shoulder", "base", "shoulder", meios::joint_kind::revolute)};
    model.topo   = meios::robot_topology{{-1, 0, 1}, {-1, 0}, {0}, {0, 1, 2}};

    return model;
}

// A joint table whose last entry names a joint index the model does not carry.
meios::model<> unresolved_joint_arm()
{
    meios::model<> model;
    model.name   = "unresolved_joint_arm";
    model.links  = {link("base"), link("shoulder"), link("wrist")};
    model.joints = {joint("shoulder", "base", "shoulder", meios::joint_kind::revolute)};
    model.topo   = meios::robot_topology{{-1, 0, 1}, {-1, 0, 9}, {0}, {0, 1, 2}};

    return model;
}

// A link whose parent index addresses nothing in the list, left out of the traversal order so that
// no walk steps onto it. The value clears the first machine word of the bit-packed leaf array, which
// a smaller out-of-range index would still land inside.
meios::model<> spur_parent_arm()
{
    meios::model<> model;
    model.name   = "spur_parent_arm";
    model.links  = {link("base"), link("shoulder"), link("spur")};
    model.joints = {joint("shoulder", "base", "shoulder", meios::joint_kind::revolute)};
    model.topo   = meios::robot_topology{{-1, 0, 4096}, {-1, 0, -1}, {0}, {0, 1}};

    return model;
}

struct refused_derivation
{
    praxis::refusal reason;
    std::string diagnosis;
};

// Several malformations share one enumerator, so the log line is the only thing that separates them;
// a case asserting the enumerator alone is satisfied by any sibling malformation of the same class.
refused_derivation refusal_of(const meios::model<> &model, const praxis::manipulator::screw_chain_options &options)
{
    praxis::tests::captured_log captured;

    const auto derived = praxis::manipulator::build_screw_chain(model, options);

    REQUIRE_FALSE(derived.has_value());

    return refused_derivation{derived.error(), captured.text()};
}

praxis::screw_axis screw(double a, double b, double c, double d, double e, double f)
{
    praxis::screw_axis axis;
    axis << a, b, c, d, e, f;

    return axis;
}

}

TEST_CASE("a fixed joint contributes no screw and folds into the home pose")
{
    const auto derived = praxis::manipulator::build_screw_chain(bounded_arm());
    REQUIRE(derived.has_value());

    const auto &chain = *derived;

    praxis::transform home = praxis::transform::Identity();
    home.block<3, 3>(0, 0) << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    home.block<3, 1>(0, 3) << 0.0, 0.5, 0.4;

    REQUIRE(chain.joint_count() == 2);
    REQUIRE(praxis::is_approx_equal(chain.home, home));
}

TEST_CASE("each screw axis is expressed in the root link frame, not the child link frame")
{
    const auto derived = praxis::manipulator::build_screw_chain(bounded_arm());
    REQUIRE(derived.has_value());

    const auto &chain = *derived;

    REQUIRE(chain.space_screws.size() == 2);
    REQUIRE(chain.space_screws[0].isApprox(screw(0.0, 1.0, 0.0, -0.3, 0.0, 0.0), praxis::default_tolerance));
    REQUIRE(chain.space_screws[1].isApprox(screw(0.0, 0.0, 0.0, 0.0, 1.0, 0.0), praxis::default_tolerance));
}

TEST_CASE("a bounded joint carries the description's own limits")
{
    const auto derived = praxis::manipulator::build_screw_chain(bounded_arm());
    REQUIRE(derived.has_value());

    const auto &chain = *derived;

    REQUIRE(praxis::is_approx_equal(chain.limits.lower_position, praxis::manipulator::to_joint_vector({-1.5, 0.0})));
    REQUIRE(praxis::is_approx_equal(chain.limits.upper_position, praxis::manipulator::to_joint_vector({1.5, 0.4})));
    REQUIRE(praxis::is_approx_equal(chain.limits.velocity, praxis::manipulator::to_joint_vector({2.0, 0.5})));
    REQUIRE(praxis::is_approx_equal(chain.limits.acceleration, praxis::manipulator::to_joint_vector({1.0, 0.25})));
}

TEST_CASE("an unbounded joint falls back to the options rather than to the zeros it parsed")
{
    praxis::manipulator::screw_chain_options options;
    options.unbounded_position = 3.0;
    options.default_velocity   = 7.0;
    options.acceleration_ratio = 0.25;

    const auto model   = quarter_turned_arm(limits{0.0, 0.0, 0.0, 0.0}, meios::joint_kind::continuous);
    const auto derived = praxis::manipulator::build_screw_chain(model, options);
    REQUIRE(derived.has_value());

    const auto &chain = *derived;

    REQUIRE(praxis::is_approx_equal(chain.limits.lower_position, praxis::manipulator::to_joint_vector({-3.0, 0.0})));
    REQUIRE(praxis::is_approx_equal(chain.limits.upper_position, praxis::manipulator::to_joint_vector({3.0, 0.4})));
    REQUIRE(praxis::is_approx_equal(chain.limits.velocity, praxis::manipulator::to_joint_vector({7.0, 0.5})));
    REQUIRE(praxis::is_approx_equal(chain.limits.acceleration, praxis::manipulator::to_joint_vector({1.75, 0.125})));
}

TEST_CASE("the walk names the actuated joints and the tip link it selected")
{
    const auto model = bounded_arm();
    const praxis::manipulator::screw_chain_options options;

    const auto names = praxis::manipulator::actuated_joint_names(model, options);
    const auto tip   = praxis::manipulator::tip_link_name(model, options);

    REQUIRE(names.has_value());
    REQUIRE(tip.has_value());
    REQUIRE(*names == std::vector<std::string>{"shoulder", "slide"});
    REQUIRE(*tip == "tool");
}

TEST_CASE("a named tip link truncates the chain to the joints above it")
{
    const auto model = bounded_arm();
    praxis::manipulator::screw_chain_options options;
    options.tip_link = "shoulder";

    const auto derived = praxis::manipulator::build_screw_chain(model, options);
    REQUIRE(derived.has_value());

    const auto &chain = *derived;

    const auto names = praxis::manipulator::actuated_joint_names(model, options);

    REQUIRE(chain.joint_count() == 1);
    REQUIRE(names.has_value());
    REQUIRE(*names == std::vector<std::string>{"shoulder"});
}

TEST_CASE("an actuated joint whose axis is the zero vector is refused as degenerate")
{
    const auto derived = praxis::manipulator::build_screw_chain(zero_axis_arm());

    REQUIRE_FALSE(derived.has_value());
    REQUIRE(derived.error() == praxis::refusal::degenerate);
}

TEST_CASE("a description with no actuated joint between root and tip is refused as unsupported input")
{
    const auto derived = praxis::manipulator::build_screw_chain(fixed_only_arm());

    REQUIRE_FALSE(derived.has_value());
    REQUIRE(derived.error() == praxis::refusal::unsupported_input);
}

TEST_CASE("a description carrying no link at all is refused as unsupported input")
{
    const auto outcome = refusal_of(link_less_arm(), praxis::manipulator::screw_chain_options());

    REQUIRE(outcome.reason == praxis::refusal::unsupported_input);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("model 'link_less_arm' has no links"));
}

TEST_CASE("a description whose every link has a parent is refused as unsupported input")
{
    const auto outcome = refusal_of(rootless_arm(), praxis::manipulator::screw_chain_options());

    REQUIRE(outcome.reason == praxis::refusal::unsupported_input);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("model 'rootless_arm' has no root link"));
}

TEST_CASE("a tip link the description does not declare is refused as unsupported input")
{
    praxis::manipulator::screw_chain_options options;
    options.tip_link = "elbow";

    const auto outcome = refusal_of(bounded_arm(), options);

    REQUIRE(outcome.reason == praxis::refusal::unsupported_input);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("has no link named 'elbow'"));
}

TEST_CASE("a description whose root reaches no leaf link is refused as unsupported input")
{
    const auto outcome = refusal_of(unreachable_root_arm(), praxis::manipulator::screw_chain_options());

    REQUIRE(outcome.reason == praxis::refusal::unsupported_input);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("has no leaf link reachable from its root"));
}

// The two cases below share an enumerator on purpose: both are traversals the parent chain cannot
// complete. A truncated walk out of the cyclic branch reaches the not-connected check and refuses
// there with the same enumerator, so only the diagnosis separates the two sites.
TEST_CASE("a tip link whose parent chain is cyclic is refused as degenerate")
{
    praxis::manipulator::screw_chain_options options;
    options.tip_link = "upper";

    const auto outcome = refusal_of(cyclic_branch_arm(), options);

    REQUIRE(outcome.reason == praxis::refusal::degenerate);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("the parent chain of link 'upper' is cyclic"));
}

TEST_CASE("a tip link not connected to the root is refused as degenerate")
{
    praxis::manipulator::screw_chain_options options;
    options.tip_link = "hand";

    const auto outcome = refusal_of(detached_arm(), options);

    REQUIRE(outcome.reason == praxis::refusal::degenerate);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("link 'hand' is not connected to root link 'base'"));
}

TEST_CASE("a link the joint table does not reach is refused rather than dereferenced")
{
    const auto outcome = refusal_of(short_joint_table_arm(), praxis::manipulator::screw_chain_options());

    REQUIRE(outcome.reason == praxis::refusal::degenerate);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("model 'short_joint_table_arm' does not address a joint above link index 2"));
}

TEST_CASE("a joint index the model does not carry is refused rather than dereferenced")
{
    const auto outcome = refusal_of(unresolved_joint_arm(), praxis::manipulator::screw_chain_options());

    REQUIRE(outcome.reason == praxis::refusal::degenerate);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("model 'unresolved_joint_arm' does not address a joint above link index 2"));
}

TEST_CASE("a parent index outside the link list is refused, reached by a walk or not")
{
    const auto outcome = refusal_of(spur_parent_arm(), praxis::manipulator::screw_chain_options());

    REQUIRE(outcome.reason == praxis::refusal::degenerate);
    REQUIRE_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring("model 'spur_parent_arm' has a parent entry that addresses no link"));
}

TEST_CASE("the walk names no joint and no tip link when the description is refused")
{
    const praxis::manipulator::screw_chain_options options;

    const auto names = praxis::manipulator::actuated_joint_names(link_less_arm(), options);
    const auto tip   = praxis::manipulator::tip_link_name(link_less_arm(), options);

    REQUIRE_FALSE(names.has_value());
    REQUIRE_FALSE(tip.has_value());
    REQUIRE(names.error() == praxis::refusal::unsupported_input);
    REQUIRE(tip.error() == praxis::refusal::unsupported_input);
}
