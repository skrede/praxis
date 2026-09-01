#include "six_axis_machine.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "captured_log.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/screw_chain_builder.h"
#include "praxis/manipulator/scene_robot_builder.h"
#include "praxis/manipulator/screw_modeling_window.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/evaluation/residual.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <span>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using opening = screw_modeling_window::settings;
using route   = screw_modeling_window::save_route;
using writer  = screw_modeling_window::edit_route;

constexpr const char *panel_title    = "Screw chain";
constexpr std::string_view screws_at = "machine/screws";

// What a point read back out of a float buffer is comparable at, and what two doubles carrying the
// same quantity by two routes are.
constexpr double read_back = 1.0e-4;
constexpr double exactly   = 1.0e-12;

// Where a joint's controls stand, counted down from the panel's first navigable item in a panel
// offering no home control and no reset. A three-component box is one stop of the walk and not
// three: the walk only steps down, so it reaches the leftmost field of a row and the rest of that
// row is stepped along.
constexpr std::size_t joint_selector = 0u; // the joints the chain has, offered as a list
constexpr std::size_t selector_row   = 1u; // the two constructions, offered as a list
constexpr std::size_t point_row      = 2u; // the point, three fields and the canonicalizing control
constexpr std::size_t direction_row  = 3u; // the direction, three fields and the normalizing control
constexpr std::size_t pitch_row      = 4u; // the pitch, one field

// A panel offering the reset control and no home control stands one lower throughout: the reset
// button is what the walk reaches first.
constexpr std::size_t reset_control = 0u;
constexpr std::size_t below_reset   = 1u;

constexpr std::size_t translating_joint = 2u;

rigid_motion::screw_ops turning()
{
    return rigid_motion::baseline().screw;
}

rigid_motion::frame_ops framing()
{
    return rigid_motion::baseline().frame;
}

forward_kinematics_ops solving()
{
    return manipulator::baseline().fk;
}

// A forward map that is not the one a second implementation would answer with, so a difference
// reading zero is one taken through this map on both sides rather than one that happens to agree
// with a product of exponentials written somewhere else.
expected<transform, refusal> lifted_forward_kinematics(const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    expected<transform, refusal> posed = forward_kinematics(m, space_screws, theta);
    if(!posed)
        return posed;

    posed.value().block<3, 1>(0, 3) += Eigen::Vector3d::UnitZ();

    return posed;
}

screw_axis unit_z_axis()
{
    screw_axis screw;
    screw << Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero();

    return screw;
}

screw_axis unit_z_direction()
{
    screw_axis screw;
    screw << Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ();

    return screw;
}

// An axis parallel to z through a point away from the origin, so the point a row is seeded with is
// one the projection had to compute rather than the zero it starts at.
screw_axis axis_through(const Eigen::Vector3d &point)
{
    const expected<screw_axis, refusal> built = turning().screw_axis_from_point_direction_pitch(point, Eigen::Vector3d::UnitZ(), 0.0);
    REQUIRE(built.has_value());

    return built.value();
}

// An axis through the origin along a direction that differs from joint to joint, so the edits a
// table mints are distinguishable from one another.
screw_axis axis_along(const Eigen::Vector3d &direction)
{
    const expected<screw_axis, refusal> built = turning().screw_axis_from_point_direction_pitch(Eigen::Vector3d::Zero(), direction, 0.0);
    REQUIRE(built.has_value());

    return built.value();
}

screw_chain described_chain()
{
    const expected<screw_chain, refusal> built = build_screw_chain(six_axis_machine());
    REQUIRE(built.has_value());

    return built.value();
}

screw_chain with_a_translating_joint(screw_chain derived, std::size_t joint)
{
    derived.space_screws[joint] << Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX();

    return derived;
}

joint_vector at_rest()
{
    return joint_vector::Zero(static_cast<Eigen::Index>(axes));
}

// Only the joints reach this window, so the rest of a publication is what an arm at rest reports.
arm_snapshot standing(const joint_vector &joints)
{
    const transform put = transform::Identity();
    const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
    const rotation level(rotation::Identity());

    return arm_snapshot{joints,
                        joint_limits{},
                        put,
                        put,
                        put,
                        origin,
                        origin,
                        level,
                        level,
                        recording_parameters{},
                        1.0,
                        false,
                        scheduler::task_counters{},
                        {},
                        unexpected(refusal::not_implemented),
                        unexpected(refusal::not_implemented),
                        jacobian_manipulability{unexpected(refusal::not_implemented), unexpected(refusal::not_implemented)},
                        jacobian_manipulability{unexpected(refusal::not_implemented), unexpected(refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

// A composition binding every screw slot but the construction each row is built through. A
// default-constructed aggregate carries the inert implementations, so the slot is left at its
// default by taking it from there.
rigid_motion::screw_ops without_the_construction()
{
    const rigid_motion::screw_ops inert;
    rigid_motion::screw_ops composed        = turning();
    composed.screw_axis_from_angular_linear = inert.screw_axis_from_angular_linear;

    return composed;
}

rigid_motion::screw_slot_set the_construction()
{
    rigid_motion::screw_slot_set held;
    held.set(rigid_motion::screw_slot::screw_axis_from_angular_linear);

    return held;
}

screw_modeling_window::controls only_the_rows()
{
    screw_modeling_window::controls offered;
    offered.home  = false;
    offered.reset = false;

    return offered;
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly. The derived chain is handed in rather than taken from the model, so a case can
// put a translating joint in one where the description carries none.
struct stage
{
    stage(screw_chain derived, const joint_vector &at)
            : stage(std::move(derived), at, turning(), rigid_motion::screw_slot_set{})
    {
    }

    stage(screw_chain derived, const joint_vector &at, const rigid_motion::screw_ops &composed, rigid_motion::screw_slot_set inert)
            : loop(praxis::scheduler::inline_workers)
            , chain(std::move(derived))
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(build_scene_robot(six_axis_machine()).value(), attached_models{}, *scene, loop.main_strand(), published->reader(), composed, inert)
    {
        published->publish(std::make_shared<const arm_snapshot>(standing(at)));
        REQUIRE(shown.initialize().has_value());
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    std::vector<Eigen::Vector3d> axis_of(std::size_t joint)
    {
        return line_in_world(*scene, loadable_robot_stencil::joint_axis_name(joint));
    }

    threepp::Color tone_of(std::size_t joint)
    {
        threepp::Object3D *drawn = first_line_under(*scene, loadable_robot_stencil::joint_axis_name(joint));
        REQUIRE(drawn != nullptr);
        const auto *shaded = drawn->materialAs<threepp::MaterialWithColor>();
        REQUIRE(shaded != nullptr);

        return shaded->color;
    }

    // Which joint the drawing was told to tell apart, read out of the scene rather than asked of the
    // window: the told joint's axis line is the one wearing a tone no other axis line wears.
    std::optional<std::size_t> told_joint()
    {
        const threepp::Color plain = tone_of(0u).equals(tone_of(1u)) ? tone_of(0u) : tone_of(2u);

        std::optional<std::size_t> found;
        for(std::size_t joint = 0u; joint < axes; ++joint)
            if(!tone_of(joint).equals(plain))
            {
                REQUIRE_FALSE(found.has_value());
                found = joint;
            }

        return found;
    }

    praxis::scheduler::scheduler loop;
    screw_chain chain;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

// The window carries no spelling of a chain, so the suite hands it one nothing in the library would
// have written: a single leaf per joint naming that joint's first component.
std::vector<config::edit> spelled(const opening &state)
{
    std::vector<config::edit> spelling;
    for(std::size_t joint = 0u; joint < state.screws.size(); ++joint)
        spelling.push_back(config::edit{std::string(screws_at) + "/joint" + std::to_string(joint + 1u), config::exact_text(state.screws[joint][0])});

    return spelling;
}

writer a_writer()
{
    return [](const config::document &, std::string_view, const opening &state) { return spelled(state); };
}

screw_modeling_window opened_over(stage &headless, const screw_modeling_window::controls &offered, const opening &state, writer edits = writer(), route save = route(),
                                  std::string at = std::string())
{
    return screw_modeling_window(panel_title, headless.shown, headless.published->reader(), turning(), framing(), solving(), headless.chain, offered, state, std::move(edits),
                                 std::move(save), std::move(at));
}

drawing over(scene::imgui_window &panel)
{
    return [&panel] { panel.render(); };
}

// Each typed value takes a context of its own: the cursor a committed box leaves behind does not
// always return to its row's first field, so a typing sharing a context with the one before it
// would be addressed from wherever that one ended.
void type_component(scene::imgui_window &panel, std::size_t below_top, std::size_t along, const char *typed)
{
    imgui_frame frames;
    const drawing draw = over(panel);

    type_into_component(frames, draw, below_top, along, typed);
}

// A control a row carries beside its fields is reached by stepping right past them, which a walk
// down the panel never does.
void press_along(scene::imgui_window &panel, std::size_t below_top, std::size_t along)
{
    imgui_frame frames;
    const drawing draw = over(panel);

    stand_below_top(frames, draw, below_top);
    for(std::size_t step = 0u; step < along; ++step)
        tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// The list opens with its keyboard cursor on its first entry rather than on the entry standing, so a
// joint is reached by stepping down from the first as many times as its position.
void select_joint(scene::imgui_window &panel, std::size_t below_top, std::size_t joint)
{
    imgui_frame frames;
    const drawing draw = over(panel);

    stand_below_top(frames, draw, below_top);
    tap(frames, draw, ImGuiKey_Space);
    for(std::size_t step = 0u; step < joint; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

std::size_t items_offered(scene::imgui_window &panel)
{
    imgui_frame frames;
    const drawing draw = over(panel);

    return navigable_items(frames, draw);
}

// Whether a point stands on the drawn segment rather than merely on the line through it, which is
// what a drawn axis reaching far enough either way of its anchor buys.
bool covers(const std::vector<Eigen::Vector3d> &drawn, const Eigen::Vector3d &at)
{
    const Eigen::Vector3d span = drawn.back() - drawn.front();
    const double along         = (at - drawn.front()).dot(span) / span.squaredNorm();
    if(along < 0.0 || along > 1.0)
        return false;

    return (drawn.front() + along * span - at).norm() < read_back;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-screw-modeling-window";
    std::filesystem::create_directories(directory);

    return directory;
}

// A document of a keyspace naming none of this window's leaves, which is what the window's own
// offer is narrowed against: a key the document declares nothing for is one it does not carry.
config::document elsewhere()
{
    config::declaration shape("probe");
    shape.group("machine");
    shape.field("machine/name", config::field_kind::text, std::string());

    const std::filesystem::path where = scratch() / "elsewhere.xml";
    std::filesystem::remove(where);
    REQUIRE(config::write_template(shape, where).has_value());

    const config::outcome answered = config::load_or_defaults(shape, config::resolve(where, scratch()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

bool same_edits(std::span<const config::edit> first, std::span<const config::edit> second)
{
    if(first.size() != second.size())
        return false;

    for(std::size_t at = 0u; at < first.size(); ++at)
        if(first[at].key != second[at].key || first[at].value != second[at].value)
            return false;

    return true;
}

}

TEST_CASE("a chain nobody supplied opens at one coincident axis through the origin per joint", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel(panel_title, headless.shown, headless.published->reader(), turning(), framing(), solving(), headless.chain);
    panel.initialize();
    headless.draw();

    const opening opened = panel.state();

    REQUIRE(opened.screws.size() == axes);
    CHECK((opened.home - transform::Identity()).norm() < exactly);
    for(const screw_axis &screw : opened.screws)
        CHECK((screw - unit_z_axis()).norm() < exactly);

    const std::vector<Eigen::Vector3d> first = headless.axis_of(0u);

    REQUIRE(first.size() == 2u);
    CHECK(first.front().head<2>().norm() < read_back);
    for(std::size_t joint = 1u; joint < axes; ++joint)
        CHECK(same_line(first, headless.axis_of(joint)));
}

TEST_CASE("a joint whose derived screw only translates opens as a direction and asks for the other numbers", "[manipulator][modeling]")
{
    stage headless(with_a_translating_joint(described_chain(), translating_joint), at_rest());
    stage all_turning(described_chain(), at_rest());

    screw_modeling_window panel  = opened_over(headless, only_the_rows(), opening{});
    screw_modeling_window beside = opened_over(all_turning, only_the_rows(), opening{});
    panel.initialize();
    beside.initialize();

    const opening opened = panel.state();

    CHECK((opened.screws[translating_joint] - unit_z_direction()).norm() < exactly);
    for(std::size_t joint = 0u; joint < axes; ++joint)
        if(joint != translating_joint)
            CHECK((opened.screws[joint] - unit_z_axis()).norm() < exactly);

    // The window draws the joint its selector names and no other, so the two panels are compared at
    // the joint whose numbers differ rather than at the one they both open on.
    select_joint(panel, joint_selector, translating_joint);
    select_joint(beside, joint_selector, translating_joint);

    CHECK(items_offered(panel) + 1u == items_offered(beside));
}

TEST_CASE("a point, a direction and a pitch typed into a row name the screw the published construction names", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});

    type_component(panel, point_row, 0u, "0.25");
    type_component(panel, point_row, 1u, "0.5");
    type_component(panel, direction_row, 0u, "1");
    type_component(panel, direction_row, 2u, "0");
    type_component(panel, pitch_row, 0u, "0.125");

    const expected<screw_axis, refusal> built = turning().screw_axis_from_point_direction_pitch(Eigen::Vector3d(0.25, 0.5, 0.0), Eigen::Vector3d::UnitX(), 0.125);

    REQUIRE(built.has_value());
    CHECK((panel.state().screws.front() - built.value()).norm() < exactly);
}

TEST_CASE("the same axis named through a different point on it leaves the drawn line where it was", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});
    panel.initialize();

    type_component(panel, point_row, 0u, "0.25");
    headless.draw();

    const std::vector<Eigen::Vector3d> first = headless.axis_of(0u);

    REQUIRE(first.size() == 2u);
    CHECK(covers(first, Eigen::Vector3d(0.25, 0.0, 0.0)));

    type_component(panel, point_row, 2u, "0.5");
    headless.draw();

    CHECK(same_line(first, headless.axis_of(0u)));
    CHECK(covers(first, Eigen::Vector3d(0.25, 0.0, 0.5)));
}

TEST_CASE("a row's selector is a choice of input, and is restored from the screw the row carries", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    std::vector<screw_axis> kept(axes, unit_z_axis());
    kept[translating_joint] = unit_z_direction();

    screw_modeling_window supplied = opened_over(headless, only_the_rows(), opening{transform::Identity(), kept});
    screw_modeling_window derived  = opened_over(headless, only_the_rows(), opening{});
    supplied.initialize();
    derived.initialize();

    select_joint(supplied, joint_selector, translating_joint);
    select_joint(derived, joint_selector, translating_joint);

    CHECK(items_offered(supplied) + 1u == items_offered(derived));

    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});
    const screw_axis opened     = panel.state().screws.front();

    imgui_frame frames;
    const drawing draw = over(panel);
    stand_below_top(frames, draw, selector_row);
    take_next_entry(frames, draw);

    CHECK((panel.state().screws.front() - opened).norm() < exactly);
}

TEST_CASE("a direction of no length names no axis, so the screw the row carried is kept and the refusal is said", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});
    const screw_axis opened     = panel.state().screws.front();

    const std::string said = reported_by([&panel] { type_component(panel, direction_row, 2u, "0"); });

    CHECK((panel.state().screws.front() - opened).norm() < exactly);
    CHECK(said.find("named no axis for joint 1") != std::string::npos);
}

TEST_CASE("the difference between the supplied chain and the derived one is taken through one bound forward map", "[manipulator][modeling]")
{
    stage headless(described_chain(), folded());
    const forward_kinematics_ops substituted{.forward_kinematics = &lifted_forward_kinematics};

    screw_modeling_window matched(panel_title, headless.shown, headless.published->reader(), turning(), framing(), substituted, headless.chain, screw_modeling_window::controls(),
                                  opening{headless.chain.home, headless.chain.space_screws}, writer(), route());
    const scene::readout agreed = matched.reading();

    REQUIRE(agreed.message.empty());
    REQUIRE(agreed.rows.size() == 2u);
    CHECK(static_cast<double>(agreed.rows.front().front().value) <= evaluation::pose_tolerance_radians);
    CHECK(static_cast<double>(agreed.rows.back().front().value) <= evaluation::pose_tolerance_metres);

    screw_modeling_window degenerate(panel_title, headless.shown, headless.published->reader(), turning(), framing(), substituted, headless.chain, screw_modeling_window::controls(),
                                     opening{}, writer(), route());
    const scene::readout apart = degenerate.reading();

    REQUIRE(apart.message.empty());
    CHECK(static_cast<double>(apart.rows.back().front().value) > evaluation::pose_tolerance_metres);
}

TEST_CASE("a window named no key path and no route offers nothing, and one named both hands the route the chain it holds", "[manipulator][configuration]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window unrouted(panel_title, headless.shown, headless.published->reader(), turning(), framing(), solving(), headless.chain);

    CHECK(unrouted.settings_path().empty());
    CHECK(unrouted.as_configurable() == nullptr);

    opening handed;
    bool reached    = false;
    const auto keep = [&handed, &reached](std::string_view, const opening &state)
    {
        handed  = state;
        reached = true;
    };

    screw_modeling_window routed  = opened_over(headless, only_the_rows(), opening{}, a_writer(), keep, std::string(screws_at));
    screw_modeling_window unsaved = opened_over(headless, only_the_rows(), opening{}, a_writer());

    const config::configurable *answered = routed.as_configurable();

    REQUIRE(answered != nullptr);
    CHECK(answered->settings_path() == screws_at);

    const std::size_t with_save = items_offered(routed);

    REQUIRE(with_save == items_offered(unsaved) + 1u);

    imgui_frame frames;
    const drawing draw = over(routed);
    stand_below_top(frames, draw, with_save - 1u);
    tap(frames, draw, ImGuiKey_Space);

    REQUIRE(reached);
    CHECK(same_edits(spelled(handed), answered->settings_edits(elsewhere())));
}

// The window has no wire format of its own to fall back on, so a composition that names a key path
// and hands no writer has named a place with nothing to put in it.
TEST_CASE("a window named a key path and handed no writer offers nothing", "[manipulator][configuration]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window named = opened_over(headless, only_the_rows(), opening{}, writer(), route(), std::string(screws_at));

    CHECK(named.settings_path() == screws_at);
    CHECK(named.as_configurable() == nullptr);
    CHECK(named.settings_edits(elsewhere()).empty());
}

// A refused row keeps the screw it carried, so the numbers on the panel have to be put back to that
// screw too: a row left showing what was refused rebuilds from it the next time any other number in
// the row moves, and refuses again for a reason the person has already corrected.
TEST_CASE("a refused row is put back to the screw it kept, so the next accepted edit builds from that", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});
    const screw_axis opened     = panel.state().screws.front();

    static_cast<void>(reported_by([&panel] { type_component(panel, direction_row, 2u, "0"); }));

    REQUIRE((panel.state().screws.front() - opened).norm() < exactly);

    type_component(panel, pitch_row, 0u, "0.1");

    CHECK((panel.state().screws.front() - opened).norm() > exactly);
    CHECK(panel.state().screws.front().head<3>().isApprox(opened.head<3>()));
}

// The table is as long as the derived chain whatever it is supplied, and a composition handing it
// more screws than the arm has loses the rest -- which the reader of a kept table refuses out loud,
// so the window says it too rather than absorbing it.
TEST_CASE("a chain supplied more screws than the arm has drops the rest and says both counts", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    const std::vector<screw_axis> too_many(axes + 2u, unit_z_axis());
    const opening supplied{transform::Identity(), too_many};

    const std::string said = reported_by([&headless, &supplied] { static_cast<void>(opened_over(headless, only_the_rows(), supplied)); });

    screw_modeling_window panel = opened_over(headless, only_the_rows(), supplied);

    CHECK(panel.state().screws.size() == axes);
    CHECK(said.find(std::to_string(axes + 2u)) != std::string::npos);
    CHECK(said.find(std::to_string(axes)) != std::string::npos);
}

TEST_CASE("every row of a freshly opened window shows the point the screw it holds carries", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    const std::vector<screw_axis> kept(axes, axis_through(Eigen::Vector3d(0.4, 0.0, 0.0)));
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{transform::Identity(), kept});

    for(std::size_t joint = 0u; joint < axes; ++joint)
        CHECK(panel.shows_stored_point(joint));
}

TEST_CASE("a point typed along the axis it already names is taken, and the row says it is not the stored one", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    const std::vector<screw_axis> kept(axes, axis_through(Eigen::Vector3d(0.4, 0.0, 0.0)));
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{transform::Identity(), kept});
    const screw_axis opened     = panel.state().screws.front();

    type_component(panel, point_row, 2u, "0.5");

    CHECK((panel.state().screws.front() - opened).norm() < read_back);
    CHECK_FALSE(panel.shows_stored_point(0u));
}

TEST_CASE("the control on the point row writes the stored point into it and leaves the axis where it was", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    const std::vector<screw_axis> kept(axes, axis_through(Eigen::Vector3d(0.4, 0.0, 0.0)));
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{transform::Identity(), kept});

    type_component(panel, point_row, 2u, "0.5");
    const screw_axis before = panel.state().screws.front();

    REQUIRE_FALSE(panel.shows_stored_point(0u));

    press_along(panel, point_row, 3u);

    CHECK((panel.state().screws.front() - before).norm() < read_back);
    CHECK(panel.shows_stored_point(0u));
}

TEST_CASE("a typed point stands where it was typed on every frame until the control is pressed", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});

    type_component(panel, point_row, 2u, "0.5");

    REQUIRE_FALSE(panel.shows_stored_point(0u));

    imgui_frame frames;
    const drawing draw = over(panel);
    frames.draw_frame(draw);
    frames.draw_frame(draw);

    CHECK_FALSE(panel.shows_stored_point(0u));
}

TEST_CASE("a refused row is put back to the point the screw it kept carries", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});

    type_component(panel, point_row, 2u, "0.5");

    REQUIRE_FALSE(panel.shows_stored_point(0u));

    static_cast<void>(reported_by([&panel] { type_component(panel, direction_row, 2u, "0"); }));

    CHECK(panel.shows_stored_point(0u));
}

TEST_CASE("a row typed as an angular and a linear part carries no point to differ", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    std::vector<screw_axis> kept(axes, unit_z_axis());
    kept[translating_joint] = unit_z_direction();

    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{transform::Identity(), kept});

    CHECK(panel.shows_stored_point(translating_joint));
    CHECK(panel.shows_stored_point(axes));
}

TEST_CASE("a point differing by less than the row's boxes print is not reported as differing", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});

    type_component(panel, point_row, 2u, "0.0004");

    CHECK(panel.shows_stored_point(0u));

    type_component(panel, point_row, 2u, "0.0006");

    CHECK_FALSE(panel.shows_stored_point(0u));
}

TEST_CASE("the chain window opens on the first joint and tells the drawing which one it is about", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});
    panel.initialize();
    headless.draw();

    REQUIRE(headless.told_joint().has_value());
    CHECK(headless.told_joint().value() == 0u);

    type_component(panel, point_row, 0u, "0.75");

    CHECK(panel.state().screws.front().tail<3>().norm() > read_back);
    for(std::size_t joint = 1u; joint < axes; ++joint)
        CHECK(panel.state().screws[joint].tail<3>().norm() < read_back);
}

TEST_CASE("taking a different entry draws that joint's row and tells the drawing that joint", "[manipulator][modeling]")
{
    constexpr std::size_t taken = 3u;

    stage headless(described_chain(), at_rest());
    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{});
    panel.initialize();

    select_joint(panel, joint_selector, taken);
    headless.draw();

    REQUIRE(headless.told_joint().has_value());
    CHECK(headless.told_joint().value() == taken);

    type_component(panel, point_row, 0u, "0.75");

    CHECK(panel.state().screws[taken].tail<3>().norm() > read_back);
    for(std::size_t joint = 0u; joint < axes; ++joint)
        if(joint != taken)
            CHECK(panel.state().screws[joint].tail<3>().norm() < read_back);
}

// Which row somebody was looking at is not a fact about a chain, so it reaches neither the table the
// window holds nor the edits it mints.
TEST_CASE("the edits the chain window mints are the same before and after the selection moves", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    std::vector<screw_axis> kept;
    for(std::size_t joint = 0u; joint < axes; ++joint)
        kept.push_back(axis_along(Eigen::Vector3d(1.0 + static_cast<double>(joint), 1.0, 1.0)));

    screw_modeling_window panel = opened_over(headless, only_the_rows(), opening{transform::Identity(), kept}, a_writer(), route(), std::string(screws_at));
    panel.initialize();

    const std::vector<config::edit> before = panel.settings_edits(elsewhere());
    REQUIRE(before.size() == axes);

    select_joint(panel, joint_selector, 4u);

    CHECK(same_edits(before, panel.settings_edits(elsewhere())));
    CHECK(same_edits(before, spelled(panel.state())));
}

TEST_CASE("resetting the chain leaves the selector naming a joint the table still has", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());
    screw_modeling_window::controls reset_only;
    reset_only.home = false;

    screw_modeling_window panel = opened_over(headless, reset_only, opening{});
    panel.initialize();
    const std::size_t offered = items_offered(panel);

    select_joint(panel, below_reset + joint_selector, axes - 1u);
    type_component(panel, below_reset + point_row, 0u, "0.75");

    REQUIRE(panel.state().screws.back().tail<3>().norm() > read_back);

    press_along(panel, reset_control, 0u);
    headless.draw();

    REQUIRE(headless.told_joint().has_value());
    CHECK(headless.told_joint().value() == 0u);
    CHECK(items_offered(panel) == offered);

    type_component(panel, below_reset + point_row, 0u, "0.75");

    CHECK(panel.state().screws.front().tail<3>().norm() > read_back);
    CHECK(panel.state().screws.back().tail<3>().norm() < read_back);
}

// Every row is built through a construction carrying no refusal channel, so a composition that left
// it at its default is answered the zero screw for each of them. A chain of zero screws is not the
// chain anybody typed and is not what the rest of the arm should be read against, so none is
// composed and the slot is named.
TEST_CASE("a window whose screw construction is unbound composes no chain and names that slot", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest(), without_the_construction(), the_construction());

    const std::string reported = reported_by(
            [&]
            {
                screw_modeling_window panel(panel_title, headless.shown, headless.published->reader(), without_the_construction(), framing(), solving(), headless.chain);
                panel.initialize();
            });

    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("screw.screw_axis_from_angular_linear"));
    CHECK_FALSE(headless.shown.holds_chain());
}

TEST_CASE("a window whose screw construction is bound composes its chain and says nothing", "[manipulator][modeling]")
{
    stage headless(described_chain(), at_rest());

    const std::string reported = reported_by(
            [&]
            {
                screw_modeling_window panel(panel_title, headless.shown, headless.published->reader(), turning(), framing(), solving(), headless.chain);
                panel.initialize();
            });

    CHECK(reported.empty());
    CHECK(headless.shown.holds_chain());
}
