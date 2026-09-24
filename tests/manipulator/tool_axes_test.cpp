#include "fixtures.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <threepp/scenes/Scene.hpp>
#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Robot.hpp>

#include <threepp/geometries/BoxGeometry.hpp>

#include <threepp/materials/MeshBasicMaterial.hpp>

#include <threepp/math/Box3.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <initializer_list>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

// The marker is sized from a world-space box, so a scene carrying a transform of its own is what
// makes "the robot is in the scene" observable: an unparented robot measures its local box instead.
constexpr float scene_scale = 2.f;
constexpr float link_edge   = 0.8f;

constexpr double marker_axis_length_fraction = 0.15;
constexpr double single_precision_tolerance  = 1.0e-5;

// The extent an arm carrying no drawn geometry is given in place of one, in metres.
constexpr double bare_arm_extent = 1.0;

// Far enough that a marker left where it stood and a marker carried along cannot be confused, and
// small beside the arm so the offset is the only thing that moved.
constexpr float tool_offset_along_x = 0.25f;

// The last joint driven from rest, in radians.
constexpr double last_joint_turn = 0.5;

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};
    return clock_source{&reading};
}

// The two-joint topology the fixture arm reports, with a box on the root link so the robot has an
// extent to be measured at all: a hierarchy of bare nodes carries no geometry and no bounding box.
std::shared_ptr<threepp::Robot> measurable_handle()
{
    auto handle = std::make_shared<threepp::Robot>();

    auto base  = threepp::Mesh::create(threepp::BoxGeometry::create(link_edge, link_edge, link_edge), threepp::MeshBasicMaterial::create());
    base->name = "base";
    handle->addLink(base);
    for(const char *name : {"upper", "tool"})
    {
        auto link  = threepp::Object3D::create();
        link->name = name;
        handle->addLink(link);
    }

    handle->addJoint(threepp::Object3D::create(), revolute("shoulder", "base", "upper"));
    handle->addJoint(threepp::Object3D::create(), revolute("elbow", "upper", "tool"));
    handle->finalize();

    return handle;
}

std::shared_ptr<threepp::Scene> scaled_scene()
{
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    target->scale.set(scene_scale, scene_scale, scene_scale);
    target->updateMatrixWorld();

    return target;
}

// Nothing under test reads a tool model's geometry, only where it is put, so a bare node stands in
// for a mesh a loader would have produced.
std::shared_ptr<threepp::Object3D> supplied_tool()
{
    return threepp::Object3D::create();
}

double extent_of(threepp::Object3D &measured)
{
    threepp::Box3 box;
    box.setFromObject(measured);

    return static_cast<double>(box.getSize().length());
}

// Each axis is drawn to the far end of its cone, so the marker's world box reaches exactly one axis
// length along x; dividing by the scale it hangs under gives back the length it was built with.
double axis_length_of(threepp::Object3D &marker, double under_scale)
{
    threepp::Box3 box;
    box.setFromObject(marker);

    return static_cast<double>(box.max().x) / under_scale;
}

// The flange transform is a world matrix, so a direction read out of it only stands beside the
// marker's where the scene the marker hangs under carries no scale of its own.
double axis_departure(const threepp::Matrix4 &flange, const threepp::Matrix4 &drawn, unsigned int axis)
{
    threepp::Vector3 held;
    held.setFromMatrixColumn(flange, axis).normalize();

    threepp::Vector3 shown;
    shown.setFromMatrixColumn(drawn, axis).normalize().sub(held);

    return std::max({std::abs(static_cast<double>(shown.x)), std::abs(static_cast<double>(shown.y)), std::abs(static_cast<double>(shown.z))});
}

// The rule every attachment is carried by, written here rather than taken from the code under test,
// so that a case comparing against it compares against the rule and not against the walk applying it.
threepp::Matrix4 carried_by(const threepp::Matrix4 &flange, const threepp::Matrix4 &offset)
{
    threepp::Matrix4 at(flange);
    at.multiply(offset);

    return at;
}

// The same pose as the renderer holds one: column by column, and in single precision. Written here
// rather than taken from the code under test, so a case comparing against it compares against the
// rule.
threepp::Matrix4 as_the_renderer_holds(const praxis::transform &placed)
{
    std::array<float, 16> held{};
    for(Eigen::Index column = 0; column < 4; ++column)
        for(Eigen::Index row = 0; row < 4; ++row)
            held[static_cast<std::size_t>(4 * column + row)] = static_cast<float>(placed(row, column));

    return threepp::Matrix4(held);
}

// A tool offset that is no coordinate translation and no coordinate rotation, so a marker standing at
// the flange instead of at the tool frame differs in position and in every axis direction at once.
praxis::transform turned_tool_offset()
{
    praxis::transform offset     = praxis::transform::Identity();
    offset.topLeftCorner<3, 3>() = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 1.0, 1.0).normalized()).toRotationMatrix();
    offset.block<3, 1>(0, 3)     = Eigen::Vector3d(0.12, -0.07, 0.19);

    return offset;
}

// Two turns compared component by component. `Quaternion::angleTo` goes through an arc cosine of a
// dot product near one, where single-precision rounding alone reads as a milliradian, so it is no
// instrument for asking whether a turn stayed put.
double turn_departure(const threepp::Quaternion &left, const threepp::Quaternion &right)
{
    return std::max({std::abs(static_cast<double>(left.x - right.x)), std::abs(static_cast<double>(left.y - right.y)), std::abs(static_cast<double>(left.z - right.z)),
                     std::abs(static_cast<double>(left.w - right.w))});
}

// How far the pose written onto a node stands from the pose the rule names, in metres of position
// and in the worst component of any axis direction.
double placement_departure(threepp::Object3D &drawn, const threepp::Matrix4 &rule)
{
    threepp::Vector3 place;
    place.setFromMatrixPosition(rule);

    threepp::Matrix4 turned;
    turned.makeRotationFromQuaternion(drawn.quaternion);

    double worst = static_cast<double>(drawn.position.distanceTo(place));
    for(unsigned int axis = 0; axis < 3; ++axis)
        worst = std::max(worst, axis_departure(rule, turned, axis));

    return worst;
}

std::shared_ptr<robot_controller> controlling(scene_robot &driven)
{
    return std::make_shared<robot_controller>(driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                                              praxis::rigid_motion::screw_ops{}, praxis::rigid_motion::frame_ops{});
}

// The order a preset composes in: the stencil is built, the windows over it are built, and only then
// is anything initialized. A measurement taken in either constructor is taken before the robot is in
// the scene, which is the whole of what the first cases below distinguish.
struct composed_arm
{
    arm_reader seen;
    std::shared_ptr<threepp::Scene> target;
    std::shared_ptr<owned_arm> owned;
    std::shared_ptr<loadable_robot_stencil> shown;
    std::shared_ptr<tool_window> panel;
};

composed_arm compose(scheduler &loop, attachments attached = attachments{}, std::shared_ptr<threepp::Scene> target = scaled_scene())
{
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto published = std::make_shared<arm_publisher>();
    const auto owned     = std::make_shared<owned_arm>(work, work, driven, controlling(*driven), published);

    auto shown = std::make_shared<loadable_robot_stencil>(measurable_handle(), std::move(attached), *target, loop.main_strand(), published->reader(),
                                                          praxis::rigid_motion::baseline().screw, praxis::rigid_motion::screw_slot_set{});

    return composed_arm{published->reader(), std::move(target), owned, shown,
                        std::make_shared<tool_window>("Tool settings", *shown, published->reader(), owned, praxis::rigid_motion::baseline().frame)};
}

// What a composition does: it asks for the marker itself, from the published builder, over the arm as
// it stands. No window installs it and no window is opened to get it.
std::shared_ptr<threepp::Object3D> ask_for_marker(composed_arm &over)
{
    over.shown->set_flange_attachment(flange_attachment::frame_marker, make_flange_marker(over.shown->robot()));

    return over.shown->attached_at(flange_attachment::frame_marker);
}

attachments under(flange_marker_policy stated)
{
    attachments named;
    named.marker = stated;

    return named;
}

std::size_t carried_at_flange(const loadable_robot_stencil &shown)
{
    std::size_t counted = 0;
    for(const flange_attachment which : {flange_attachment::tool, flange_attachment::frame_marker})
        if(shown.attached_at(which) != nullptr)
            ++counted;

    return counted;
}

void draw_frame(scheduler &loop, composed_arm &over)
{
    REQUIRE(loop.main_strand().post([&over] { over.shown->render(); }).has_value());
    REQUIRE(loop.drain().has_value());
}

void tell_tool_offset(scheduler &loop, composed_arm &over, const praxis::transform &offset)
{
    const std::weak_ptr<owned_arm> observer = over.owned;
    command(observer, [offset](robot_controller &, scene_robot &driven) { driven.set_tool_offset(offset); });
    REQUIRE(loop.drain().has_value());
}

void drive_last_joint(scheduler &loop, composed_arm &over, double angle)
{
    const std::weak_ptr<owned_arm> observer = over.owned;
    const joint_vector commanded            = configuration(0.0, angle);
    command(observer, [commanded](robot_controller &, scene_robot &driven) { driven.set_joint_positions(commanded); });
    REQUIRE(loop.drain().has_value());
}

}

TEST_CASE("the frame marker is sized from the robot's extent as the scene holds it, not as it stood before", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop);
    REQUIRE(placed.shown->initialize().has_value());
    placed.panel->initialize();

    const double seated                          = extent_of(placed.shown->robot());
    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    REQUIRE(seated > 0.0);
    REQUIRE(put != nullptr);
    CHECK(std::abs(axis_length_of(*put, scene_scale) - marker_axis_length_fraction * seated) < single_precision_tolerance);

    composed_arm unplaced = compose(loop);
    unplaced.panel->initialize();

    const double loose                            = extent_of(unplaced.shown->robot());
    const std::shared_ptr<threepp::Object3D> away = ask_for_marker(unplaced);
    REQUIRE(std::abs(loose - seated) > single_precision_tolerance);
    CHECK(std::abs(axis_length_of(*away, scene_scale) - marker_axis_length_fraction * loose) < single_precision_tolerance);

    placed.shown->tear_down();
}

// An arm of bare nodes has no bounding box, so a proportion of its extent is a proportion of nothing.
// The builder answers a marker at the stated stand-in rather than one of no size at all.
TEST_CASE("a marker asked for over an arm carrying no drawn geometry is built at the stand-in extent", "[manipulator][tool]")
{
    const std::shared_ptr<threepp::Robot> bare = two_joint_handle();

    threepp::Box3 around;
    around.setFromObject(*bare);
    REQUIRE(around.isEmpty());

    const std::shared_ptr<threepp::Object3D> put = make_flange_marker(*bare);
    REQUIRE(put != nullptr);
    CHECK(axis_length_of(*put, 1.0) > 0.0);
    CHECK(std::abs(axis_length_of(*put, 1.0) - marker_axis_length_fraction * bare_arm_extent) < single_precision_tolerance);
}

TEST_CASE("a composition asking for the frame marker shows it and still drives the arm", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop);
    REQUIRE(placed.shown->attached_at(flange_attachment::frame_marker) == nullptr);
    REQUIRE(placed.shown->initialize().has_value());

    placed.panel->initialize();
    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);

    REQUIRE(put != nullptr);
    CHECK(put->parent == placed.target.get());
    CHECK(axis_length_of(*put, scene_scale) > 0.0);

    const std::weak_ptr<owned_arm> observer = placed.owned;
    const joint_vector commanded            = configuration(0.3, -0.2);
    command(observer, [commanded](robot_controller &, scene_robot &driven) { driven.set_joint_positions(commanded); });
    REQUIRE(loop.drain().has_value());

    CHECK(placed.seen.read()->joints.isApprox(commanded, round_trip));

    placed.shown->tear_down();
}

TEST_CASE("the frame marker is drawn along the flange's own axes, one direction at a time", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, attachments{}, threepp::Scene::create());
    REQUIRE(placed.shown->initialize().has_value());
    placed.panel->initialize();

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    REQUIRE(put != nullptr);
    REQUIRE(put->parent == placed.target.get());
    draw_frame(loop, placed);

    const threepp::Matrix4 flange = placed.shown->robot().getEndEffectorTransform();
    threepp::Matrix4 drawn;
    drawn.makeRotationFromQuaternion(put->quaternion);

    const double along_x = axis_departure(flange, drawn, 0);
    const double along_y = axis_departure(flange, drawn, 1);
    const double along_z = axis_departure(flange, drawn, 2);

    CHECK(along_x < single_precision_tolerance);
    CHECK(along_y < single_precision_tolerance);
    CHECK(along_z < single_precision_tolerance);

    placed.shown->tear_down();
}

// The invariant the flange's attachment set exists to hold. It goes red the moment a marker is put
// back into the tool key: the window saying "no tool attached" would take the marker away with it.
TEST_CASE("with no tool attached and the frame marker asked for, the flange carries exactly one marker", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop);
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    REQUIRE(put != nullptr);

    placed.panel->initialize();

    CHECK(placed.shown->attached_at(flange_attachment::tool) == nullptr);
    CHECK(placed.shown->attached_at(flange_attachment::frame_marker) == put);
    CHECK(carried_at_flange(*placed.shown) == 1);
    CHECK(put->parent == placed.target.get());

    placed.shown->tear_down();
}

TEST_CASE("attaching a tool beside the frame marker leaves the flange carrying both", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop);
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    placed.panel->initialize();
    REQUIRE(carried_at_flange(*placed.shown) == 1);

    const std::shared_ptr<threepp::Object3D> held = supplied_tool();
    placed.shown->set_flange_attachment(flange_attachment::tool, held);

    CHECK(placed.shown->attached_at(flange_attachment::tool) == held);
    CHECK(placed.shown->attached_at(flange_attachment::frame_marker) == put);
    CHECK(carried_at_flange(*placed.shown) == 2);
    CHECK(put->parent == placed.target.get());
    CHECK(held->parent == placed.target.get());

    placed.shown->tear_down();
}

TEST_CASE("clearing the tool leaves the frame marker exactly where it was", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, attachments{}, threepp::Scene::create());
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    placed.shown->set_flange_attachment(flange_attachment::tool, supplied_tool());
    draw_frame(loop, placed);

    const threepp::Vector3 stood     = put->position;
    const threepp::Quaternion turned = put->quaternion;

    placed.shown->clear_flange_attachment(flange_attachment::tool);
    draw_frame(loop, placed);

    CHECK(placed.shown->attached_at(flange_attachment::frame_marker) == put);
    CHECK(carried_at_flange(*placed.shown) == 1);
    CHECK(put->parent == placed.target.get());
    CHECK(static_cast<double>(put->position.distanceTo(stood)) < single_precision_tolerance);
    CHECK(turn_departure(put->quaternion, turned) < single_precision_tolerance);

    placed.shown->tear_down();
}

TEST_CASE("under the standing policy the frame marker is drawn with a tool seated and with the tool vacated", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, under(flange_marker_policy::stands));
    REQUIRE(placed.shown->initialize().has_value());
    REQUIRE(placed.shown->flange_marker_policy_held() == flange_marker_policy::stands);

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    draw_frame(loop, placed);
    REQUIRE(put->visible);

    placed.shown->set_flange_attachment(flange_attachment::tool, supplied_tool());
    draw_frame(loop, placed);
    CHECK(put->visible);

    placed.shown->clear_flange_attachment(flange_attachment::tool);
    draw_frame(loop, placed);
    CHECK(put->visible);

    placed.shown->tear_down();
}

// The tool comes and goes all session, so the rule is read where every attachment is already placed
// rather than answered once: two cycles say the marker returns rather than merely leaves.
TEST_CASE("under the yielding policy the frame marker is withheld while a tool is seated and drawn again once it is vacated", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, under(flange_marker_policy::yields));
    REQUIRE(placed.shown->initialize().has_value());
    REQUIRE(placed.shown->flange_marker_policy_held() == flange_marker_policy::yields);

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    draw_frame(loop, placed);
    REQUIRE(put->visible);

    for(const int cycle : {1, 2})
    {
        INFO("seat and vacate " << cycle);
        placed.shown->set_flange_attachment(flange_attachment::tool, supplied_tool());
        draw_frame(loop, placed);
        CHECK_FALSE(put->visible);

        placed.shown->clear_flange_attachment(flange_attachment::tool);
        draw_frame(loop, placed);
        CHECK(put->visible);
    }

    placed.shown->tear_down();
}

// What a yielding policy changes is whether the marker is drawn and nothing else: the flange carries
// it, answers it and parents it exactly as it did before the tool arrived.
TEST_CASE("a frame marker a yielding policy withholds is withheld rather than detached", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, under(flange_marker_policy::yields));
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    placed.shown->set_flange_attachment(flange_attachment::tool, supplied_tool());
    draw_frame(loop, placed);

    CHECK_FALSE(put->visible);
    CHECK(placed.shown->attached_at(flange_attachment::frame_marker) == put);
    CHECK(carried_at_flange(*placed.shown) == 2);
    CHECK(put->parent == placed.target.get());

    placed.shown->tear_down();
}

TEST_CASE("the marker switch withholds the frame marker under the standing policy and returns it", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, under(flange_marker_policy::stands));
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    draw_frame(loop, placed);
    REQUIRE(put->visible);

    placed.shown->set_flange_marker_shown(false);
    draw_frame(loop, placed);
    CHECK_FALSE(put->visible);

    placed.shown->set_flange_marker_shown(true);
    draw_frame(loop, placed);
    CHECK(put->visible);

    placed.shown->tear_down();
}

// The two reasons to withhold the marker are independent and either suffices, so a switch turned on
// reaches only as far as the policy admits.
TEST_CASE("a switch turned on does not bring the frame marker back while a yielding policy withholds it", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, under(flange_marker_policy::yields));
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    placed.shown->set_flange_attachment(flange_attachment::tool, supplied_tool());
    placed.shown->set_flange_marker_shown(false);
    draw_frame(loop, placed);
    REQUIRE_FALSE(put->visible);

    placed.shown->set_flange_marker_shown(true);
    draw_frame(loop, placed);
    CHECK_FALSE(put->visible);

    placed.shown->clear_flange_attachment(flange_attachment::tool);
    draw_frame(loop, placed);
    CHECK(put->visible);

    placed.shown->tear_down();
}

TEST_CASE("a stencil told no marker policy stands the marker beside a tool, as one told the standing policy does", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm untold = compose(loop);
    composed_arm told   = compose(loop, under(flange_marker_policy::stands));
    REQUIRE(untold.shown->initialize().has_value());
    REQUIRE(told.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> left  = ask_for_marker(untold);
    const std::shared_ptr<threepp::Object3D> right = ask_for_marker(told);
    untold.shown->set_flange_attachment(flange_attachment::tool, supplied_tool());
    told.shown->set_flange_attachment(flange_attachment::tool, supplied_tool());
    draw_frame(loop, untold);
    draw_frame(loop, told);

    CHECK(untold.shown->flange_marker_policy_held() == told.shown->flange_marker_policy_held());
    CHECK(left->visible == right->visible);
    CHECK(left->visible);

    untold.shown->tear_down();
    told.shown->tear_down();
}

// One rule for the whole set: the flange's pose composed with the attachment's own offset. Giving one
// attachment an offset is therefore a statement about that attachment and about nothing else.
TEST_CASE("every flange attachment is carried by the flange's pose composed with its own offset", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, attachments{}, threepp::Scene::create());
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put  = ask_for_marker(placed);
    const std::shared_ptr<threepp::Object3D> held = supplied_tool();
    placed.shown->set_flange_attachment(flange_attachment::tool, held);
    draw_frame(loop, placed);

    const threepp::Matrix4 identity;
    const threepp::Matrix4 flange = placed.shown->robot().getEndEffectorTransform();
    CHECK(placement_departure(*put, carried_by(flange, identity)) < single_precision_tolerance);
    CHECK(placement_departure(*held, carried_by(flange, identity)) < single_precision_tolerance);

    const threepp::Vector3 stood = put->position;

    threepp::Matrix4 aside;
    aside.setPosition(tool_offset_along_x, 0.f, 0.f);
    placed.shown->set_flange_attachment_offset(flange_attachment::tool, aside);
    draw_frame(loop, placed);

    CHECK(placement_departure(*held, carried_by(flange, aside)) < single_precision_tolerance);
    CHECK(static_cast<double>(held->position.distanceTo(stood)) > static_cast<double>(tool_offset_along_x) / 2.0);
    CHECK(static_cast<double>(put->position.distanceTo(stood)) < single_precision_tolerance);

    placed.shown->tear_down();
}

// The cue that the last joint turns, which is what the marker exists at the flange for. It has to
// hold with no tool model loaded at all, which is the configuration a reader taking the default sees.
TEST_CASE("driving the last joint turns the frame marker with no tool attached", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, attachments{}, threepp::Scene::create());
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> put = ask_for_marker(placed);
    placed.panel->initialize();
    REQUIRE(placed.shown->attached_at(flange_attachment::tool) == nullptr);

    drive_last_joint(loop, placed, 0.0);
    draw_frame(loop, placed);
    const threepp::Quaternion before = put->quaternion;

    drive_last_joint(loop, placed, last_joint_turn);
    draw_frame(loop, placed);

    CHECK(static_cast<double>(put->quaternion.angleTo(before)) > last_joint_turn / 2.0);
    CHECK(placement_departure(*put, carried_by(placed.shown->robot().getEndEffectorTransform(), threepp::Matrix4())) < single_precision_tolerance);

    placed.shown->tear_down();
}

TEST_CASE("a tool window opens at the view its settings name, and at the loader when it holds no tool", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    const Eigen::Vector3f none = Eigen::Vector3f::Zero();
    const Eigen::Vector3f unit = Eigen::Vector3f{1.f, 1.f, 1.f};
    const tool_window::settings graphics{true, "models/tool.stl", tool_window::tool_view::graphics_transform, none, praxis::axis_order::zyx, unit, none, none, praxis::axis_order::zyx,
                                         none};
    const attachments shaped{threepp::Mesh::create(threepp::BoxGeometry::create(link_edge, link_edge, link_edge), threepp::MeshBasicMaterial::create()), nullptr};

    composed_arm seated = compose(loop, shaped);
    tool_window carried("Tool settings", *seated.shown, seated.seen, seated.owned, praxis::rigid_motion::baseline().frame, graphics, "machine/tool");
    carried.initialize();

    composed_arm bare = compose(loop);
    tool_window loose("Tool settings", *bare.shown, bare.seen, bare.owned, praxis::rigid_motion::baseline().frame, graphics, "machine/tool");
    loose.initialize();

    CHECK(carried.state().selected_view == tool_window::tool_view::graphics_transform);
    CHECK(loose.state().selected_view == tool_window::tool_view::load_stl);

    seated.shown->tear_down();
    bare.shown->tear_down();
}

// The frame the arm's own tool offset defines, not one whoever installed the marker keeps in step with
// it: the offset is read from what the arm published, so telling the arm a new one is the whole of
// what moves the marker. The marker at the flange is the control: it is carried at its own offset and
// stays where it was.
TEST_CASE("the marker at the tool frame stands at the offset the arm published, at a tool offset about no coordinate axis", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, attachments{}, threepp::Scene::create());
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> at_the_flange = ask_for_marker(placed);
    placed.shown->set_flange_attachment(flange_attachment::tool_frame_marker, make_flange_marker(placed.shown->robot()));
    const std::shared_ptr<threepp::Object3D> at_the_tool = placed.shown->attached_at(flange_attachment::tool_frame_marker);
    REQUIRE(at_the_tool != nullptr);

    draw_frame(loop, placed);
    const threepp::Matrix4 flange = placed.shown->robot().getEndEffectorTransform();
    const threepp::Matrix4 identity;
    REQUIRE(placement_departure(*at_the_tool, carried_by(flange, identity)) < single_precision_tolerance);

    const praxis::transform offset = turned_tool_offset();
    tell_tool_offset(loop, placed, offset);
    draw_frame(loop, placed);

    CHECK(placement_departure(*at_the_tool, carried_by(flange, as_the_renderer_holds(offset))) < single_precision_tolerance);
    CHECK(placement_departure(*at_the_flange, carried_by(flange, identity)) < single_precision_tolerance);
    CHECK(static_cast<double>(at_the_tool->position.distanceTo(at_the_flange->position)) > offset.block<3, 1>(0, 3).norm() / 2.0);

    placed.shown->tear_down();
}

// One multiple over every frame the flange marks, so a document naming it once reaches both markers.
// A marker is built at a size proportioned to the arm, and the multiple is read on the size drawn
// rather than on the number the stencil holds.
TEST_CASE("the marker size multiple reaches the size every marker the flange carries is drawn at", "[manipulator][tool]")
{
    scheduler loop(inline_workers, dictating());

    composed_arm placed = compose(loop, attachments{}, threepp::Scene::create());
    REQUIRE(placed.shown->initialize().has_value());

    const std::shared_ptr<threepp::Object3D> at_the_flange = ask_for_marker(placed);
    placed.shown->set_flange_attachment(flange_attachment::tool_frame_marker, make_flange_marker(placed.shown->robot()));
    const std::shared_ptr<threepp::Object3D> at_the_tool = placed.shown->attached_at(flange_attachment::tool_frame_marker);
    REQUIRE(at_the_tool != nullptr);

    draw_frame(loop, placed);
    const double flange_built = extent_of(*at_the_flange);
    const double tool_built   = extent_of(*at_the_tool);
    REQUIRE(placed.shown->marker_scale() == 1.0);
    REQUIRE(flange_built > 0.0);

    REQUIRE(placed.shown->set_marker_scale(2.5).has_value());
    draw_frame(loop, placed);

    CHECK(placed.shown->marker_scale() == 2.5);
    CHECK(std::abs(extent_of(*at_the_flange) - 2.5 * flange_built) < single_precision_tolerance);
    CHECK(std::abs(extent_of(*at_the_tool) - 2.5 * tool_built) < single_precision_tolerance);

    CHECK_FALSE(placed.shown->set_marker_scale(0.0).has_value());
    CHECK_FALSE(placed.shown->set_marker_scale(-1.0).has_value());
    draw_frame(loop, placed);

    CHECK(placed.shown->marker_scale() == 2.5);
    CHECK(std::abs(extent_of(*at_the_flange) - 2.5 * flange_built) < single_precision_tolerance);

    placed.shown->tear_down();
}
