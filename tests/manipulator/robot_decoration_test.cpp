#include "fixtures.h"
#include "drawn_chain.h"
#include "captured_log.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/extension/coverage.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <string_view>
#include <initializer_list>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

// The nine screw slots answering a plain value. Nothing folded through one of them can learn from
// its return that the composition never bound it, which is why the decline over an unbound slot is
// decided from the composition's own bindings and not from what the fold answered.
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::skew_symmetric)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::from_skew_symmetric)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::twist_from_angular_linear)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::twist_matrix_from_angular_linear)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::twist_matrix_from_twist)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::screw_axis_from_angular_linear)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::matrix_exponential_so3)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::matrix_exponential_se3)>);
static_assert(!praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::matrix_exponential_screw)>);

// The adjoint map is the refusal-carrying slot the fold above reaches, and it stands beside them so
// that the predicate is read telling the two kinds apart rather than answering false everywhere.
static_assert(praxis::returns_refusal_v<decltype(praxis::rigid_motion::screw_ops::adjoint_map)>);

// The half-length the decoration opens at against an arm carrying no drawn geometry, in metres, and
// what a point read back out of a float buffer and turned out of the renderer's world is comparable
// at.
constexpr double opening_reach = 1.0;
constexpr double read_back     = 1.0e-4;

// Only the configuration is read by what is under test, but a snapshot has no field a publication
// may leave out, so the rest is filled with what an arm at rest reports.
arm_snapshot at_rest(const joint_vector &joints)
{
    const praxis::transform put = praxis::transform::Identity();
    const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
    const praxis::rotation upright(praxis::rotation::Identity());

    return arm_snapshot{joints,
                        joint_limits{},
                        put,
                        put,
                        put,
                        origin,
                        origin,
                        upright,
                        upright,
                        recording_parameters{},
                        1.0,
                        false,
                        task_counters{},
                        {},
                        praxis::unexpected(praxis::refusal::not_implemented),
                        praxis::unexpected(praxis::refusal::not_implemented),
                        jacobian_manipulability{praxis::unexpected(praxis::refusal::not_implemented), praxis::unexpected(praxis::refusal::not_implemented)},
                        jacobian_manipulability{praxis::unexpected(praxis::refusal::not_implemented), praxis::unexpected(praxis::refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

praxis::screw_axis revolute_screw(const Eigen::Vector3d &through, const Eigen::Vector3d &along)
{
    return praxis::rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(through, along, 0.0).value();
}

// A point lies on the axis a screw names exactly where the screw's own linear part is what the
// angular part crossed into that point gives up, which is the relation the axis is defined by.
bool on_axis(const praxis::screw_axis &named, const Eigen::Vector3d &at)
{
    return (named.head<3>().cross(at) + named.tail<3>()).norm() < read_back;
}

// The tone a drawn item wears is the color of the material it was handed, which is the readable fact
// a still of the scene would show.
threepp::Color tone_of(threepp::Object3D *drawn)
{
    REQUIRE(drawn != nullptr);
    const auto *shaded = drawn->materialAs<threepp::MaterialWithColor>();
    REQUIRE(shaded != nullptr);

    return shaded->color;
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly.
struct stage
{
    explicit stage(const joint_vector &joints)
            : stage(joints, praxis::rigid_motion::baseline().screw, praxis::rigid_motion::screw_slot_set{})
    {
    }

    stage(const joint_vector &joints, const praxis::rigid_motion::screw_ops &turning, praxis::rigid_motion::screw_slot_set inert)
            : loop(inline_workers)
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), turning, inert)
    {
        publish(joints);
        REQUIRE(shown.initialize().has_value());
    }

    void publish(const joint_vector &joints)
    {
        published->publish(std::make_shared<const arm_snapshot>(at_rest(joints)));
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    std::vector<Eigen::Vector3d> axis(std::size_t joint)
    {
        return line_in_world(*scene, loadable_robot_stencil::joint_axis_name(joint));
    }

    std::vector<Eigen::Vector3d> chain()
    {
        return chain_in_world(*scene, loadable_robot_stencil::chain_name(), &loadable_robot_stencil::chain_segment_name);
    }

    threepp::Object3D *axis_node(std::size_t joint)
    {
        return first_line_under(*scene, loadable_robot_stencil::joint_axis_name(joint));
    }

    threepp::Object3D *chain_node()
    {
        return praxis::fixture::chain_node(*scene, loadable_robot_stencil::chain_name());
    }

    threepp::Object3D *segment_node(std::size_t segment)
    {
        return chain_part(*scene, loadable_robot_stencil::chain_name(), loadable_robot_stencil::chain_segment_name(segment));
    }

    threepp::Object3D *mark_node(std::size_t joint)
    {
        return chain_part(*scene, loadable_robot_stencil::chain_name(), loadable_robot_stencil::joint_mark_name(joint));
    }

    threepp::Color axis_tone(std::size_t joint)
    {
        return tone_of(axis_node(joint));
    }

    threepp::Color segment_tone(std::size_t segment)
    {
        return tone_of(segment_node(segment));
    }

    threepp::Color mark_tone(std::size_t joint)
    {
        return tone_of(mark_node(joint));
    }

    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

praxis::screw_axis upright_at_the_origin()
{
    return revolute_screw(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
}

std::vector<praxis::screw_axis> two_axes()
{
    return {upright_at_the_origin(), revolute_screw(Eigen::Vector3d(static_cast<double>(link_length), 0.0, 0.0), Eigen::Vector3d::UnitZ())};
}

// Every screw slot bound but the ones named, which is what a composition that supplied some of the
// capability and none of the rest hands over. A default-constructed aggregate carries the inert
// implementations, so a slot is left at its default by taking it from there.
praxis::rigid_motion::screw_ops bound_without(std::initializer_list<praxis::rigid_motion::screw_slot> left)
{
    const praxis::rigid_motion::screw_ops inert;
    praxis::rigid_motion::screw_ops turning = praxis::rigid_motion::baseline().screw;
    for(praxis::rigid_motion::screw_slot slot : left)
    {
        if(slot == praxis::rigid_motion::screw_slot::matrix_exponential_screw)
            turning.matrix_exponential_screw = inert.matrix_exponential_screw;
        if(slot == praxis::rigid_motion::screw_slot::adjoint_map)
            turning.adjoint_map = inert.adjoint_map;
    }

    return turning;
}

// How many times a report carries one name, which is what tells a line said once for as long as a
// condition stands from a line said once a frame.
std::size_t said_of(const std::string &reported, std::string_view named)
{
    std::size_t times = 0u;
    for(std::string::size_type at = reported.find(named); at != std::string::npos; at = reported.find(named, at + named.size()))
        ++times;

    return times;
}

praxis::rigid_motion::screw_slot_set slots(std::initializer_list<praxis::rigid_motion::screw_slot> named)
{
    praxis::rigid_motion::screw_slot_set held;
    for(praxis::rigid_motion::screw_slot slot : named)
        held.set(slot);

    return held;
}

// A stage whose composition left the named slots at their defaults, told the two axes and drawn
// once, which is the state every case below reads.
struct unbound_stage
{
    explicit unbound_stage(std::initializer_list<praxis::rigid_motion::screw_slot> left)
            : headless(configuration(0.3, -0.4), bound_without(left), slots(left))
    {
        REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    }

    stage headless;
};

}

TEST_CASE("a stencil told one screw per joint draws one line per joint, lying along the axis that screw names", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    const std::vector<praxis::screw_axis> told = two_axes();

    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), told).has_value());
    headless.draw();

    for(std::size_t joint = 0; joint < told.size(); ++joint)
    {
        const std::vector<Eigen::Vector3d> ends = headless.axis(joint);
        REQUIRE(ends.size() == 2u);
        CHECK(on_axis(told[joint], ends.front()));
        CHECK(on_axis(told[joint], ends.back()));
        CHECK(std::abs((ends.back() - ends.front()).norm() - 2.0 * opening_reach) < read_back);
    }
}

// A screw axis is a line rather than a place, and the line reaching well past the joint either way
// is what makes that visible instead of teaching that an axis lives somewhere.
TEST_CASE("two screws naming one axis through different points of it draw the same line", "[manipulator][decoration]")
{
    const Eigen::Vector3d along   = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
    const Eigen::Vector3d through = Eigen::Vector3d(0.2, -0.4, 0.15);

    stage nearer(configuration(0.0, 0.0));
    REQUIRE(nearer.shown.set_joint_screws(praxis::transform::Identity(), std::vector<praxis::screw_axis>(2, revolute_screw(through, along))).has_value());
    nearer.draw();

    stage further(configuration(0.0, 0.0));
    REQUIRE(further.shown.set_joint_screws(praxis::transform::Identity(), std::vector<praxis::screw_axis>(2, revolute_screw(through + 1.75 * along, along))).has_value());
    further.draw();

    CHECK(same_line(nearer.axis(0), further.axis(0)));
    CHECK(same_line(nearer.axis(1), further.axis(1)));
}

TEST_CASE("a stencil told a count of screws that is not the rendered arm's refuses by name and leaves the decoration alone", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> before = headless.axis(0);
    const std::vector<praxis::screw_axis> three(3, upright_at_the_origin());
    praxis::expected<void, praxis::refusal> answered{};

    const std::string reported = praxis::tests::reported_by([&] { answered = headless.shown.set_joint_screws(praxis::transform::Identity(), three); });

    REQUIRE_FALSE(answered.has_value());
    CHECK(answered.error() == praxis::refusal::unsupported_input);
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("2") && Catch::Matchers::ContainsSubstring("3"));

    headless.draw();

    CHECK(same_line(headless.axis(0), before));
    CHECK(headless.axis(2).empty());
}

// The rendered arm's second joint stands a link length along x whatever it is told, so a decoration
// taking its placement from the model rather than from the screws would draw the two lines apart.
TEST_CASE("a stencil told the state every screw shares draws lines that coincide at the origin", "[manipulator][decoration]")
{
    stage headless(configuration(0.25, -0.5));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), std::vector<praxis::screw_axis>(2, upright_at_the_origin())).has_value());
    headless.draw();

    REQUIRE(headless.axis(1).size() == 2u);
    CHECK(same_line(headless.axis(0), headless.axis(1)));
    for(const Eigen::Vector3d &at : headless.axis(1))
        CHECK(at.head<2>().norm() < read_back);
}

TEST_CASE("turning a joint moves the axis of every joint its motion carries and leaves the others where they were", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> shoulder = headless.axis(0);
    const std::vector<Eigen::Vector3d> elbow    = headless.axis(1);

    headless.publish(configuration(std::numbers::pi / 2.0, 0.0));
    headless.draw();

    CHECK(same_line(headless.axis(0), shoulder));
    CHECK_FALSE(same_line(headless.axis(1), elbow));
}

// The rendered arm is the independent reference a wrong chain is read against, so the two must be
// reachable together; this is the case that fails the moment the decoration hangs under the arm.
TEST_CASE("with the meshes hidden the screw axes and the chain are still drawn and still carry their points", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> before = headless.axis(0);
    const std::vector<Eigen::Vector3d> figure = headless.chain();
    REQUIRE(figure.size() == 4u);

    headless.shown.set_meshes_shown(false);
    headless.draw();

    CHECK_FALSE(drawn(rendered_arm(*headless.scene)));
    CHECK(drawn(headless.axis_node(0)));
    CHECK(drawn(headless.chain_node()));
    CHECK(same_line(headless.axis(0), before));
    CHECK(same_line(headless.chain(), figure));
}

TEST_CASE("hiding the screw axes leaves the chain drawn, and hiding the chain leaves the screw axes drawn", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());

    headless.shown.set_decoration_shown(false);
    headless.draw();

    CHECK_FALSE(drawn(headless.axis_node(0)));
    CHECK(drawn(headless.chain_node()));

    headless.shown.set_decoration_shown(true);
    headless.shown.set_chain_shown(false);
    headless.draw();

    CHECK(drawn(headless.axis_node(0)));
    CHECK_FALSE(drawn(headless.chain_node()));
}

TEST_CASE("every combination of the three switches leaves the scene in the state those three name", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());

    for(int combination = 0; combination < 8; ++combination)
    {
        const bool meshes = (combination & 1) != 0;
        const bool axes   = (combination & 2) != 0;
        const bool stick  = (combination & 4) != 0;

        headless.shown.set_meshes_shown(meshes);
        headless.shown.set_decoration_shown(axes);
        headless.shown.set_chain_shown(stick);
        headless.draw();

        CHECK(drawn(rendered_arm(*headless.scene)) == meshes);
        CHECK(drawn(headless.axis_node(0)) == axes);
        CHECK(drawn(headless.chain_node()) == stick);
    }
}

// The written-out points are the frame's origin, the point of each axis nearest the point before it
// and the translation the home transform names, none of which the rendered arm was asked for.
TEST_CASE("the chain drawn between the joint origins is the told screws folded rather than the arm read", "[manipulator][decoration]")
{
    praxis::transform reaching = praxis::transform::Identity();
    reaching(0, 3)             = 2.0 * static_cast<double>(link_length);

    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(reaching, two_axes()).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> figure = headless.chain();
    REQUIRE(figure.size() == 4u);

    const std::vector<Eigen::Vector3d> folded{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {static_cast<double>(link_length), 0.0, 0.0}, {2.0 * static_cast<double>(link_length), 0.0, 0.0}};

    for(std::size_t at = 0; at < folded.size(); ++at)
        CHECK((figure[at] - folded[at]).norm() < read_back);
}

TEST_CASE("turning a joint moves the chain, and the configuration it was drawn at returns it", "[manipulator][decoration]")
{
    praxis::transform reaching = praxis::transform::Identity();
    reaching(0, 3)             = 2.0 * static_cast<double>(link_length);

    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(reaching, two_axes()).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> before = headless.chain();

    headless.publish(configuration(std::numbers::pi / 2.0, 0.0));
    headless.draw();

    const std::vector<Eigen::Vector3d> turned = headless.chain();
    CHECK_FALSE(same_line(turned, before));
    CHECK((turned[2] - Eigen::Vector3d(0.0, static_cast<double>(link_length), 0.0)).norm() < read_back);
    CHECK((turned[3] - Eigen::Vector3d(0.0, 2.0 * static_cast<double>(link_length), 0.0)).norm() < read_back);

    headless.publish(configuration(0.0, 0.0));
    headless.draw();

    CHECK(same_line(headless.chain(), before));
}

// The axis line and the chain polyline are two drawings of one screw and have to agree about whether
// it names an axis at all. The line is read by dividing the linear part by the angular part's norm,
// so below the shared threshold the quotient stands further out than the renderer's single-precision
// transform can carry: the node's own matrix goes infinite and every world matrix below it with it,
// while the fold draws the same screw at the point before it.
TEST_CASE("a screw whose angular part is below the shared threshold is drawn as naming no axis", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    praxis::screw_axis vanishing;
    vanishing << Eigen::Vector3d(0.0, 0.0, 1.0e-150), Eigen::Vector3d(0.0, 1.0, 0.0);

    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), std::vector<praxis::screw_axis>{vanishing, upright_at_the_origin()}).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> line = headless.axis(0);
    REQUIRE(line.size() == 2u);
    for(const Eigen::Vector3d &at : line)
    {
        REQUIRE(at.allFinite());
        CHECK(at.norm() < 2.0 * opening_reach + read_back);
    }

    const std::vector<Eigen::Vector3d> folded = headless.chain();
    REQUIRE_FALSE(folded.empty());
    for(const Eigen::Vector3d &at : folded)
        CHECK(at.allFinite());
}

TEST_CASE("a stencil told one screw per joint carries one segment per gap between chain points and one mark per joint origin", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    CHECK(headless.segment_node(0) != nullptr);
    CHECK(headless.segment_node(1) != nullptr);
    CHECK(headless.segment_node(2) != nullptr);
    CHECK(headless.segment_node(3) == nullptr);

    CHECK(headless.mark_node(0) != nullptr);
    CHECK(headless.mark_node(1) != nullptr);
    CHECK(headless.mark_node(2) == nullptr);
}

// The placement is what is read, not the primitive: each segment is asked where its own ends were
// carried to, and the answers have to be the pair of folded points that segment stands between.
TEST_CASE("each segment's ends are the two folded points it spans, and each mark stands on its joint's point", "[manipulator][decoration]")
{
    praxis::transform reaching = praxis::transform::Identity();
    reaching(0, 3)             = 2.0 * static_cast<double>(link_length);

    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(reaching, two_axes()).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> folded{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {static_cast<double>(link_length), 0.0, 0.0}, {2.0 * static_cast<double>(link_length), 0.0, 0.0}};

    for(std::size_t at = 0; at + 1 < folded.size(); ++at)
    {
        REQUIRE(headless.segment_node(at) != nullptr);
        const std::vector<Eigen::Vector3d> ends = segment_ends(*headless.segment_node(at));
        REQUIRE(ends.size() == 2u);
        CHECK((ends.front() - folded[at]).norm() < read_back);
        CHECK((ends.back() - folded[at + 1]).norm() < read_back);
    }

    for(std::size_t joint = 0; joint < 2u; ++joint)
    {
        REQUIRE(headless.mark_node(joint) != nullptr);
        CHECK((mark_in_world(*headless.mark_node(joint)) - folded[joint + 1]).norm() < read_back);
    }
}

TEST_CASE("turning a joint moves the segments its motion carries and leaves the others where they were", "[manipulator][decoration]")
{
    praxis::transform reaching = praxis::transform::Identity();
    reaching(0, 3)             = 2.0 * static_cast<double>(link_length);

    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(reaching, two_axes()).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> base    = segment_ends(*headless.segment_node(0));
    const std::vector<Eigen::Vector3d> upper   = segment_ends(*headless.segment_node(1));
    const std::vector<Eigen::Vector3d> reached = segment_ends(*headless.segment_node(2));

    headless.publish(configuration(0.0, std::numbers::pi / 2.0));
    headless.draw();

    CHECK(same_line(segment_ends(*headless.segment_node(0)), base));
    CHECK(same_line(segment_ends(*headless.segment_node(1)), upper));
    CHECK_FALSE(same_line(segment_ends(*headless.segment_node(2)), reached));

    headless.publish(configuration(0.0, 0.0));
    headless.draw();

    CHECK(same_line(segment_ends(*headless.segment_node(2)), reached));
}

// A pair of axes that meet gives the fold one point twice, which is a real answer and not a
// degenerate one: the segment between them has nothing to span and still has to read back as the
// two coincident points it stands between.
TEST_CASE("two chain points that coincide give a segment of no length, placed at that point and not drawn", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    REQUIRE(headless.segment_node(0) != nullptr);
    CHECK_FALSE(headless.segment_node(0)->visible);

    const std::vector<Eigen::Vector3d> met = segment_ends(*headless.segment_node(0));
    REQUIRE(met.size() == 2u);
    CHECK((met.front() - met.back()).norm() < read_back);
    CHECK(met.front().norm() < read_back);

    REQUIRE(headless.segment_node(1) != nullptr);
    CHECK(headless.segment_node(1)->visible);
}

TEST_CASE("a rebuild over the joint count already standing leaves the segments and the marks that are already there", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    threepp::Object3D *segment = headless.segment_node(0);
    threepp::Object3D *mark    = headless.mark_node(0);
    REQUIRE(segment != nullptr);
    REQUIRE(mark != nullptr);

    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), std::vector<praxis::screw_axis>(2, upright_at_the_origin())).has_value());
    headless.shown.set_decoration_reach(0.5);
    headless.draw();

    CHECK(headless.segment_node(0) == segment);
    CHECK(headless.mark_node(0) == mark);
}

// The chain gained extent so that a zero-width line has nothing left to hide inside; the axes are
// the thing it hides inside, and they must not have gained any of their own.
TEST_CASE("the screw axes carry no girth while the chain is drawn as solids", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    for(std::size_t joint = 0; joint < 2u; ++joint)
    {
        threepp::Object3D *axis = headless.scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::joint_axis_name(joint));
        REQUIRE(axis != nullptr);
        axis->traverse([](threepp::Object3D &at) { CHECK(at.type() == "Line"); });
    }

    REQUIRE(headless.segment_node(0) != nullptr);
    REQUIRE(headless.mark_node(0) != nullptr);
    CHECK(headless.segment_node(0)->type() == "Mesh");
    CHECK(headless.mark_node(0)->type() == "Mesh");
}

TEST_CASE("a stencil nobody has told a selection draws every joint's point, segment and axis in the two tones the drawing otherwise uses", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    const threepp::Color axes  = headless.axis_tone(0);
    const threepp::Color chain = headless.segment_tone(0);

    CHECK_FALSE(axes.equals(chain));
    CHECK(headless.axis_tone(1).equals(axes));
    for(std::size_t at = 0; at < 3u; ++at)
        CHECK(headless.segment_tone(at).equals(chain));
    for(std::size_t joint = 0; joint < 2u; ++joint)
        CHECK(headless.mark_tone(joint).equals(chain));
}

TEST_CASE("a stencil told a joint draws that joint's point, the segment leading to it and its axis in a third tone and leaves every other item alone", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    const threepp::Color axes  = headless.axis_tone(1);
    const threepp::Color chain = headless.segment_tone(1);

    REQUIRE(headless.shown.set_selected_joint(0u).has_value());
    headless.draw();

    const threepp::Color told = headless.axis_tone(0);

    CHECK_FALSE(told.equals(axes));
    CHECK_FALSE(told.equals(chain));
    CHECK(headless.segment_tone(0).equals(told));
    CHECK(headless.mark_tone(0).equals(told));

    CHECK(headless.axis_tone(1).equals(axes));
    CHECK(headless.segment_tone(1).equals(chain));
    CHECK(headless.segment_tone(2).equals(chain));
    CHECK(headless.mark_tone(1).equals(chain));
}

TEST_CASE("told a different joint the third tone moves with it and nothing is left behind wearing it", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    REQUIRE(headless.shown.set_selected_joint(0u).has_value());
    headless.draw();

    const threepp::Color told  = headless.axis_tone(0);
    const threepp::Color axes  = headless.axis_tone(1);
    const threepp::Color chain = headless.segment_tone(1);

    REQUIRE(headless.shown.set_selected_joint(1u).has_value());
    headless.draw();

    CHECK(headless.axis_tone(1).equals(told));
    CHECK(headless.segment_tone(1).equals(told));
    CHECK(headless.mark_tone(1).equals(told));

    CHECK(headless.axis_tone(0).equals(axes));
    CHECK(headless.segment_tone(0).equals(chain));
    CHECK(headless.mark_tone(0).equals(chain));
}

TEST_CASE("a stencil told to clear the selection leaves no item wearing the third tone", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    const threepp::Color axes  = headless.axis_tone(0);
    const threepp::Color chain = headless.segment_tone(0);

    REQUIRE(headless.shown.set_selected_joint(1u).has_value());
    headless.draw();

    REQUIRE_FALSE(headless.axis_tone(1).equals(axes));

    headless.shown.clear_selected_joint();
    headless.draw();

    for(std::size_t joint = 0; joint < 2u; ++joint)
    {
        CHECK(headless.axis_tone(joint).equals(axes));
        CHECK(headless.mark_tone(joint).equals(chain));
    }
    for(std::size_t at = 0; at < 3u; ++at)
        CHECK(headless.segment_tone(at).equals(chain));
}

TEST_CASE("a stencil told a joint the arm does not have declines by name and leaves the drawing exactly as it was", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    const threepp::Color axes  = headless.axis_tone(0);
    const threepp::Color chain = headless.segment_tone(0);
    praxis::expected<void, praxis::refusal> answered{};

    const std::string reported = praxis::tests::reported_by([&] { answered = headless.shown.set_selected_joint(4u); });

    REQUIRE_FALSE(answered.has_value());
    CHECK(answered.error() == praxis::refusal::unsupported_input);
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("2") && Catch::Matchers::ContainsSubstring("5"));

    headless.draw();

    for(std::size_t joint = 0; joint < 2u; ++joint)
    {
        CHECK(headless.axis_tone(joint).equals(axes));
        CHECK(headless.mark_tone(joint).equals(chain));
    }
    for(std::size_t at = 0; at < 3u; ++at)
        CHECK(headless.segment_tone(at).equals(chain));
}

// A tone is a switch and not a rebuild: the objects standing before the selection moved have to be
// the objects standing after it, or the placement would be handing the renderer new nodes per frame.
TEST_CASE("a selection that moves allocates no geometry", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    headless.draw();

    threepp::Object3D *axis    = headless.axis_node(1);
    threepp::Object3D *segment = headless.segment_node(1);
    threepp::Object3D *mark    = headless.mark_node(1);
    REQUIRE(axis != nullptr);
    REQUIRE(segment != nullptr);
    REQUIRE(mark != nullptr);

    REQUIRE(headless.shown.set_selected_joint(1u).has_value());
    headless.draw();

    CHECK(headless.axis_node(1) == axis);
    CHECK(headless.segment_node(1) == segment);
    CHECK(headless.mark_node(1) == mark);

    headless.shown.clear_selected_joint();
    headless.draw();

    CHECK(headless.axis_node(1) == axis);
    CHECK(headless.segment_node(1) == segment);
    CHECK(headless.mark_node(1) == mark);
}

// The selection is kept across a rebuild and applied to whatever the new drawing carries, so a
// window that changed a screw does not have to say again which joint it is about.
TEST_CASE("a selection told before a new set of screws is in force against the drawing they build", "[manipulator][decoration]")
{
    stage headless(configuration(0.0, 0.0));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    REQUIRE(headless.shown.set_selected_joint(1u).has_value());
    headless.draw();

    const threepp::Color told  = headless.axis_tone(1);
    const threepp::Color axes  = headless.axis_tone(0);
    const threepp::Color chain = headless.segment_tone(0);

    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), std::vector<praxis::screw_axis>(2, upright_at_the_origin())).has_value());
    headless.draw();

    CHECK(headless.axis_tone(1).equals(told));
    CHECK(headless.segment_tone(1).equals(told));
    CHECK(headless.mark_tone(1).equals(told));
    CHECK(headless.axis_tone(0).equals(axes));
    CHECK(headless.segment_tone(0).equals(chain));
}

// A slot carrying no refusal channel answers a value whatever it is handed, so an unbound screw
// exponential folds every configuration onto the home pose and the drawing stands still while the
// arm moves. The name asserted is the one the screw capability's own descriptor table spells.
TEST_CASE("a drawing folded through an unbound screw exponential is withheld and the slot is named", "[manipulator][decoration]")
{
    unbound_stage over({praxis::rigid_motion::screw_slot::matrix_exponential_screw});

    const std::string reported = praxis::tests::reported_by([&] { over.headless.draw(); });

    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("screw.matrix_exponential_screw"));
    CHECK_FALSE(over.headless.chain_node()->visible);
    CHECK_FALSE(over.headless.axis_node(0)->visible);
    CHECK_FALSE(over.headless.axis_node(1)->visible);
}

TEST_CASE("an unbound screw exponential is named once for as long as it stands and again after it returns", "[manipulator][decoration]")
{
    unbound_stage over({praxis::rigid_motion::screw_slot::matrix_exponential_screw});

    const std::string standing = praxis::tests::reported_by(
            [&]
            {
                for(int frame = 0; frame < 5; ++frame)
                    over.headless.draw();
            });

    CHECK(said_of(standing, "screw.matrix_exponential_screw") == 1u);

    const std::string lifted = praxis::tests::reported_by(
            [&]
            {
                over.headless.shown.clear_joint_screws();
                over.headless.draw();
            });

    CHECK(said_of(lifted, "screw.matrix_exponential_screw") == 0u);

    const std::string returned = praxis::tests::reported_by(
            [&]
            {
                REQUIRE(over.headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
                over.headless.draw();
            });

    CHECK(said_of(returned, "screw.matrix_exponential_screw") == 1u);
}

// A reading the composition never bound and one a capability declined are different facts, and the
// unbound answer is the earlier of the two: the fold that would have produced the refusal is not
// reached, so the adjoint map is never named.
TEST_CASE("a fold whose exponential and whose adjoint map are both unbound names the exponential", "[manipulator][decoration]")
{
    unbound_stage over({praxis::rigid_motion::screw_slot::matrix_exponential_screw, praxis::rigid_motion::screw_slot::adjoint_map});

    const std::string reported = praxis::tests::reported_by([&] { over.headless.draw(); });

    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("screw.matrix_exponential_screw"));
    CHECK_THAT(reported, !Catch::Matchers::ContainsSubstring("screw.adjoint_map"));
}

TEST_CASE("a composition binding every screw slot draws the chain and the axes and says nothing", "[manipulator][decoration]")
{
    stage headless(configuration(0.3, -0.4));
    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());

    const std::string reported = praxis::tests::reported_by([&] { headless.draw(); });

    CHECK(reported.empty());
    CHECK(headless.chain_node()->visible);
    CHECK(headless.axis_node(0)->visible);
    CHECK(headless.axis_node(1)->visible);
    CHECK(headless.chain().size() == 4u);
}
