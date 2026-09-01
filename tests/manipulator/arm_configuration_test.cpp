#include "window_stage.h"

#include "panel_labels.h"

#include "praxis/manipulator/configuration.h"
#include "praxis/manipulator/edited_list_window.h"
#include "praxis/manipulator/path_comparison_window.h"
#include "praxis/manipulator/trajectory_configuration.h"
#include "praxis/manipulator/trajectory_preview_window.h"
#include "praxis/manipulator/trajectory_recording_window.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>
#include <threepp/objects/Mesh.hpp>

#include <threepp/geometries/BoxGeometry.hpp>

#include <threepp/materials/MeshBasicMaterial.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <string_view>
#include <type_traits>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr std::string_view at               = "machine/tool";
constexpr std::string_view world_at         = "machine/world_object";
constexpr std::string_view tool_jog_at      = "machine/tool_jog";
constexpr std::string_view screw_jog_at     = "machine/screw_jog";
constexpr std::string_view parameters_at    = "machine/parameters";
constexpr std::string_view recording_at     = "machine/recording";
constexpr std::string_view task_space_at    = "machine/task_space";
constexpr std::string_view joint_control_at = "machine/joint_control";
constexpr std::string_view joint_curves_at  = "machine/joint_curves";
constexpr std::string_view comparison_at    = "machine/path_comparison";

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_tool(shape, at);
    declare_world_object(shape, world_at);
    declare_joint_control(shape, joint_control_at);
    declare_task_space(shape, task_space_at);
    declare_tool_jog(shape, tool_jog_at);
    declare_screw_jog(shape, screw_jog_at);
    declare_control_parameters(shape, parameters_at);
    declare_recording(shape, recording_at);
    declare_joint_curves(shape, joint_curves_at);
    declare_path_comparison(shape, comparison_at);
    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-arm-configuration";
    std::filesystem::create_directories(directory);
    return directory;
}

// A starter document written from the declaration alone, so every field arrives at the fallback the
// mapping declared for it rather than at anything a case wrote.
config::document starter(const std::string &name)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);

    REQUIRE(config::write_template(described(), where).has_value());

    const config::outcome answered = config::load_or_defaults(described(), config::resolve(where, scratch()));
    REQUIRE_FALSE(answered.failure.has_value());
    return answered.values;
}

// A document carrying nothing at all, which is what a machine document an author left out of the
// tree reads as: every value arrives at the fallback the declaration named.
config::document nothing_carried(const std::string &name)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);

    return config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial).values;
}

std::filesystem::path authored(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << body;
    return where;
}

// The edits written into a starter document and the document loaded back, which is the round trip
// every case here is over.
config::document saved_and_reloaded(const std::string &name, const std::vector<config::edit> &changes)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);
    REQUIRE(config::write_template(described(), where).has_value());

    const config::location at_file            = config::resolve(where, scratch());
    const expected<void, config::error> saved = config::save(described(), at_file, changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());

    const config::outcome answered = config::load_or_defaults(described(), at_file);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());
    return answered.values;
}

// The four groups saved in one document, which is how a machine document carries them.
std::vector<config::edit> every_edit_of(const joint_control_window::settings &joints, const task_space_window::settings &task, const tool_jog_window::settings &jog,
                                        const screw_jog_window::settings &screw)
{
    std::vector<config::edit> written = write_joint_control(joints, joint_control_at);
    for(const std::vector<config::edit> &group : {write_task_space(task, task_space_at), write_tool_jog(jog, tool_jog_at), write_screw_jog(screw, screw_jog_at)})
        written.insert(written.end(), group.begin(), group.end());

    return written;
}

void require_same(const Eigen::Vector3f &read, const Eigen::Vector3f &written)
{
    REQUIRE(read.x() == written.x());
    REQUIRE(read.y() == written.y());
    REQUIRE(read.z() == written.z());
}

void require_same_edits(const std::vector<config::edit> &answered, const std::vector<config::edit> &written)
{
    REQUIRE(answered.size() == written.size());
    for(std::size_t which = 0u; which < written.size(); ++which)
    {
        INFO(written[which].key);
        REQUIRE(answered[which].key == written[which].key);
        REQUIRE(answered[which].value == written[which].value);
    }
}

// A declaration inherited rather than overridden carries the base class in its member-pointer type,
// so that type names the class that answers. The two pure virtuals need no such check: a window
// leaving either unanswered would be abstract and could not be built anywhere.
template<typename window>
constexpr bool answers_with_itself()
{
    return std::is_same_v<decltype(&window::as_configurable), const config::configurable *(window::*)() const>;
}

arm_reader unattached()
{
    const arm_publisher published;

    return published.reader();
}

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

// Each of the four control windows refuses a publisher that has published nothing, so a case
// driving one stands over a publication rather than over `unattached()`.
std::shared_ptr<arm_publisher> published_arm()
{
    return publishing(at_rest(configuration(0.0, 0.0), Eigen::Vector3d::Zero(), rotation::Identity()));
}

// A stencil standing over a scene of its own and holding whatever meshes a case hands it. Nothing is
// put into the scene: what a window offers is read from the window, not from what is drawn.
struct standing_stencil
{
    std::shared_ptr<arm_publisher> published;
    std::shared_ptr<threepp::Scene> target;
    std::shared_ptr<loadable_robot_stencil> shown;
};

standing_stencil stand(scheduler::scheduler &loop, const attached_models &held)
{
    const auto published                         = std::make_shared<arm_publisher>();
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();

    return standing_stencil{published, target,
                            std::make_shared<loadable_robot_stencil>(two_joint_handle(), held, *target, loop.main_strand(), published->reader(), rigid_motion::baseline().screw,
                                                                     rigid_motion::screw_slot_set{})};
}

std::shared_ptr<threepp::Object3D> a_mesh()
{
    return threepp::Mesh::create(threepp::BoxGeometry::create(0.1f, 0.1f, 0.1f), threepp::MeshBasicMaterial::create());
}

std::string key_under(std::string_view group, std::string_view leaf)
{
    return std::string(group) + "/" + std::string(leaf);
}

void require_one_edit(const std::vector<config::edit> &offered, const std::string &key, std::string_view value)
{
    REQUIRE(offered.size() == 1u);
    REQUIRE(offered.front().key == key);
    REQUIRE(offered.front().value == value);
}

}

TEST_CASE("the tool settings read back from a starter document at every declared fallback", "[manipulator][configuration]")
{
    const config::document carried   = starter("tool-starter.xml");
    const tool_window::settings read = read_tool(carried, at);

    REQUIRE(carried.origin_of(key_under(at, "active")).kind == config::origin_kind::source);
    REQUIRE(carried.origin_of(key_under(at, "model")).kind == config::origin_kind::source);
    REQUIRE(carried.origin_of(key_under(at, "view")).kind == config::origin_kind::source);
    REQUIRE_FALSE(read.active);
    REQUIRE(read.model_path.empty());
    REQUIRE(read.selected_view == tool_window::tool_view::load_stl);
    require_same(read.gfx_euler_degrees, Eigen::Vector3f::Zero());
    REQUIRE(read.gfx_euler_order == axis_order::zyx);
    require_same(read.gfx_scale, Eigen::Vector3f{1.f, 1.f, 1.f});
    require_same(read.gfx_offset, Eigen::Vector3f::Zero());
    require_same(read.kinematics_euler_degrees, Eigen::Vector3f::Zero());
    REQUIRE(read.kinematics_euler_order == axis_order::zyx);
    require_same(read.kinematics_offset, Eigen::Vector3f::Zero());
}

TEST_CASE("every tool field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const tool_window::settings written{
            false,
            "models/other.stl",
            tool_window::tool_view::load_stl,
            Eigen::Vector3f{1.5f, -2.25f, 30.f},
            axis_order::xyz,
            Eigen::Vector3f{0.5f, 2.f, 4.f},
            Eigen::Vector3f{0.1f, 0.2f, 0.3f},
            Eigen::Vector3f{-10.f, 20.f, -30.f},
            axis_order::yzy,
            Eigen::Vector3f{9.f, 8.f, 7.f},
    };

    const tool_window::settings read = read_tool(saved_and_reloaded("tool-round-trip.xml", write_tool(written, at)), at);

    REQUIRE(read.active == written.active);
    REQUIRE(read.model_path == written.model_path);
    REQUIRE(read.selected_view == written.selected_view);
    require_same(read.gfx_euler_degrees, written.gfx_euler_degrees);
    REQUIRE(read.gfx_euler_order == written.gfx_euler_order);
    require_same(read.gfx_scale, written.gfx_scale);
    require_same(read.gfx_offset, written.gfx_offset);
    require_same(read.kinematics_euler_degrees, written.kinematics_euler_degrees);
    REQUIRE(read.kinematics_euler_order == written.kinematics_euler_order);
    require_same(read.kinematics_offset, written.kinematics_offset);
}

TEST_CASE("each vector component is a key of its own", "[manipulator][configuration]")
{
    const std::vector<config::edit> changes = write_tool(read_tool(starter("tool-keys.xml"), at), at);

    std::vector<std::string> named;
    for(const config::edit &one : changes)
        named.push_back(one.key);

    REQUIRE(std::find(named.begin(), named.end(), "machine/tool/graphics/scale/x") != named.end());
    REQUIRE(std::find(named.begin(), named.end(), "machine/tool/graphics/scale/y") != named.end());
    REQUIRE(std::find(named.begin(), named.end(), "machine/tool/graphics/scale/z") != named.end());
}

TEST_CASE("every world reference field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const world_object_window::settings written{
            false,
            "models/bench.stl",
            world_object_window::world_view::load_stl,
            Eigen::Vector3f{0.25f, 0.5f, 0.75f},
            Eigen::Vector3f{-1.5f, 2.5f, 3.5f},
            Eigen::Vector3f{15.f, -45.f, 60.f},
    };

    const world_object_window::settings read = read_world_object(saved_and_reloaded("world-round-trip.xml", write_world_object(written, world_at)), world_at);

    REQUIRE(read.active == written.active);
    REQUIRE(read.model_path == written.model_path);
    REQUIRE(read.selected_view == written.selected_view);
    require_same(read.gfx_scale, written.gfx_scale);
    require_same(read.gfx_offset, written.gfx_offset);
    require_same(read.gfx_euler_zyx_degrees, written.gfx_euler_zyx_degrees);
}

TEST_CASE("the world reference settings read back from a starter document at every declared fallback", "[manipulator][configuration]")
{
    const config::document carried           = starter("world-starter.xml");
    const world_object_window::settings read = read_world_object(carried, world_at);

    REQUIRE(carried.origin_of(key_under(world_at, "active")).kind == config::origin_kind::source);
    REQUIRE(carried.origin_of(key_under(world_at, "model")).kind == config::origin_kind::source);
    REQUIRE(carried.origin_of(key_under(world_at, "view")).kind == config::origin_kind::source);
    REQUIRE_FALSE(read.active);
    REQUIRE(read.model_path.empty());
    REQUIRE(read.selected_view == world_object_window::world_view::load_stl);
    require_same(read.gfx_scale, Eigen::Vector3f{1.f, 1.f, 1.f});
    require_same(read.gfx_offset, Eigen::Vector3f::Zero());
    require_same(read.gfx_euler_zyx_degrees, Eigen::Vector3f::Zero());
}

TEST_CASE("every operator control field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const joint_control_window::settings joints{control_mode::preview};
    const task_space_window::settings task{task_space_window::motion_shape::lin, control_mode::preview};
    const tool_jog_window::settings jog{control_mode::preview};
    const screw_jog_window::settings screw{control_mode::preview};

    const config::document carried = saved_and_reloaded("controls-round-trip.xml", every_edit_of(joints, task, jog, screw));

    REQUIRE(read_joint_control(carried, joint_control_at).mode == joints.mode);
    REQUIRE(read_task_space(carried, task_space_at).shape == task.shape);
    REQUIRE(read_task_space(carried, task_space_at).mode == task.mode);
    REQUIRE(read_tool_jog(carried, tool_jog_at).mode == jog.mode);
    REQUIRE(read_screw_jog(carried, screw_jog_at).mode == screw.mode);
}

TEST_CASE("the operator control settings read back from a starter document at every declared fallback", "[manipulator][configuration]")
{
    const config::document carried = starter("controls-starter.xml");

    REQUIRE(read_task_space(carried, task_space_at).shape == task_space_window::motion_shape::ptp);
    REQUIRE(read_joint_control(carried, joint_control_at).mode == control_mode::simulation);
    REQUIRE(read_task_space(carried, task_space_at).mode == control_mode::simulation);
    REQUIRE(read_tool_jog(carried, tool_jog_at).mode == control_mode::simulation);
    REQUIRE(read_screw_jog(carried, screw_jog_at).mode == control_mode::simulation);
}

// Each window's group stands under a key path of its own, so a value one window saves is not a value
// another window's read answers with.
TEST_CASE("a mode saved by one control window leaves the other three at what they were", "[manipulator][configuration]")
{
    const config::document carried = saved_and_reloaded("controls-one-moved.xml", write_tool_jog(tool_jog_window::settings{control_mode::preview}, tool_jog_at));

    REQUIRE(read_tool_jog(carried, tool_jog_at).mode == control_mode::preview);
    REQUIRE(read_joint_control(carried, joint_control_at).mode == control_mode::simulation);
    REQUIRE(read_task_space(carried, task_space_at).mode == control_mode::simulation);
    REQUIRE(read_screw_jog(carried, screw_jog_at).mode == control_mode::simulation);
}

// The declaration is the whole authority over what a document may carry, so a group it does not name
// refuses the document rather than being passed over -- and the refusal is over the whole document,
// which leaves the values it does declare on their fallbacks. What a document is expected to carry
// decides where a message about an absent value lands and nothing else, so a partial binding does
// not soften this.
TEST_CASE("a group the declaration does not name refuses the whole document by name", "[manipulator][configuration]")
{
    const std::filesystem::path where = authored("retired-group.xml", "<probe><machine><parameters velocity_factor=\"0.75\"/><controls shape=\"lin\"/></machine></probe>\n");

    const config::outcome answered = config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial);

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == config::error_code::rejected_content);
    REQUIRE(answered.failure->message.find("machine/controls/shape") != std::string::npos);
    REQUIRE(read_control_parameters(answered.values, parameters_at).velocity == 0.3f);
}

TEST_CASE("the control parameter written through the declared key reads back as it was set", "[manipulator][configuration]")
{
    const control_parameters_window::settings written{0.625f};

    const control_parameters_window::settings read =
            read_control_parameters(saved_and_reloaded("parameters-round-trip.xml", write_control_parameters(written, parameters_at)), parameters_at);

    REQUIRE(read.velocity == written.velocity);
    REQUIRE(read_control_parameters(starter("parameters-starter.xml"), parameters_at).velocity == 0.3f);
}

TEST_CASE("the chosen scaling and its overridden bounds read back as they were set", "[manipulator][configuration]")
{
    const control_parameters_window::settings written{0.625f, time_scaling_choice::trapezoidal, path_parameter_bounds{2.5, 10.0}};

    const control_parameters_window::settings read =
            read_control_parameters(saved_and_reloaded("scaling-round-trip.xml", write_control_parameters(written, parameters_at)), parameters_at);

    REQUIRE(read.scaling == time_scaling_choice::trapezoidal);
    REQUIRE(read.trapezoid.has_value());
    REQUIRE(read.trapezoid->max_rate == 2.5);
    REQUIRE(read.trapezoid->max_rate_change == 10.0);
}

// An absent value is the absence of a choice rather than a choice, so a document naming neither the
// scaling nor a bound opens where every scenario opened before either could be written.
TEST_CASE("a document naming no scaling opens at the quintic holding no bounds", "[manipulator][configuration]")
{
    const control_parameters_window::settings started = read_control_parameters(starter("scaling-starter.xml"), parameters_at);
    const control_parameters_window::settings absent  = read_control_parameters(nothing_carried("scaling-absent.xml"), parameters_at);

    REQUIRE(started.scaling == time_scaling_choice::quintic);
    REQUIRE_FALSE(started.trapezoid.has_value());
    REQUIRE(absent.scaling == time_scaling_choice::quintic);
    REQUIRE_FALSE(absent.trapezoid.has_value());
}

TEST_CASE("every recording field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    recording_parameters written;
    written.active    = true;
    written.directory = "recordings/session";

    const recording_parameters read = read_recording(saved_and_reloaded("recording-round-trip.xml", write_recording(written, recording_at)), recording_at);

    REQUIRE(read.active == written.active);
    REQUIRE(read.directory == written.directory);
}

TEST_CASE("a recording directory is carried as written rather than resolved", "[manipulator][configuration]")
{
    recording_parameters written;
    written.directory = "../beside/the/document";

    const recording_parameters read = read_recording(saved_and_reloaded("recording-verbatim.xml", write_recording(written, recording_at)), recording_at);

    REQUIRE(read.directory == std::filesystem::path("../beside/the/document"));
}

TEST_CASE("a value outside an enumerated field's declared set is refused by name", "[manipulator][configuration]")
{
    struct outside
    {
        std::string named;
        std::string body;
        std::string offered;
    };

    const std::vector<outside> refused{
            {"tool view", "<tool view=\"sideways\"/>", "sideways"},
            {"tool graphics euler order", "<tool><graphics euler_order=\"QQQ\"/></tool>", "QQQ"},
            {"tool kinematics euler order", "<tool><kinematics euler_order=\"QQQ\"/></tool>", "QQQ"},
            {"world reference view", "<world_object view=\"sideways\"/>", "sideways"},
            {"task space shape", "<task_space shape=\"spiral\"/>", "spiral"},
            {"task space mode", "<task_space mode=\"rehearsal\"/>", "rehearsal"},
            {"joint control mode", "<joint_control mode=\"rehearsal\"/>", "rehearsal"},
            {"tool jog mode", "<tool_jog mode=\"rehearsal\"/>", "rehearsal"},
            {"screw jog mode", "<screw_jog mode=\"rehearsal\"/>", "rehearsal"},
    };

    for(const outside &one : refused)
    {
        INFO(one.named);
        const std::filesystem::path where = authored("outside-" + one.offered + "-" + one.named.substr(0, 4) + ".xml", "<probe><machine>" + one.body + "</machine></probe>\n");

        const expected<config::document, config::error> loaded = config::load(described(), config::resolve(where, scratch()));

        REQUIRE_FALSE(loaded.has_value());
        REQUIRE(loaded.error().code == config::error_code::rejected_content);
        REQUIRE(loaded.error().message.find(one.offered) != std::string::npos);
    }
}

TEST_CASE("every settings-carrying window answers for itself", "[manipulator][configuration]")
{
    REQUIRE(answers_with_itself<tool_window>());
    REQUIRE(answers_with_itself<world_object_window>());
    REQUIRE(answers_with_itself<joint_control_window>());
    REQUIRE(answers_with_itself<task_space_window>());
    REQUIRE(answers_with_itself<tool_jog_window>());
    REQUIRE(answers_with_itself<screw_jog_window>());
    REQUIRE(answers_with_itself<control_parameters_window>());
    REQUIRE(answers_with_itself<trajectory_recording_window>());
    REQUIRE(answers_with_itself<joint_waypoint_list>());
    REQUIRE(answers_with_itself<pose_waypoint_list>());
    REQUIRE(answers_with_itself<trajectory_preview_window>());
    REQUIRE(answers_with_itself<joint_curve_window>());
}

TEST_CASE("what a window answers is what its own mapping writes for the state it shows", "[manipulator][configuration]")
{
    const config::document carried = starter("routed.xml");

    const control_parameters_window::settings rate{0.625f};
    const control_parameters_window playback("Playback", unattached(), std::weak_ptr<owned_arm>(), rate, std::string(parameters_at));
    const config::configurable *routed_rate = playback.as_configurable();

    REQUIRE(routed_rate != nullptr);
    REQUIRE(routed_rate->settings_path() == parameters_at);
    require_same_edits(routed_rate->settings_edits(carried), config::unsaved_edits(carried, write_control_parameters(rate, parameters_at)));

    recording_parameters taken;
    taken.active    = true;
    taken.directory = "recordings/session";

    const trajectory_recording_window recording("Recording", unattached(), std::weak_ptr<owned_arm>(), taken, std::string(recording_at));
    const config::configurable *routed_recording = recording.as_configurable();

    REQUIRE(routed_recording != nullptr);
    REQUIRE(routed_recording->settings_path() == recording_at);
    require_same_edits(routed_recording->settings_edits(carried), write_recording(taken, recording_at));
}

TEST_CASE("a window nobody touched offers nothing to save, whether its document carries every value or does not exist, and one whose value the operator moved offers that value alone",
          "[manipulator][configuration]")
{
    const config::document written = starter("untouched.xml");
    const config::document nothing = nothing_carried("never-authored.xml");

    const control_parameters_window playback("Playback", unattached(), std::weak_ptr<owned_arm>(), read_control_parameters(written, parameters_at), std::string(parameters_at));
    const config::configurable *routed_rate = playback.as_configurable();

    REQUIRE(routed_rate != nullptr);
    REQUIRE(routed_rate->settings_edits(written).empty());

    const trajectory_recording_window recording("Recording", unattached(), std::weak_ptr<owned_arm>(), read_recording(written, recording_at), std::string(recording_at));
    const config::configurable *routed_recording = recording.as_configurable();

    REQUIRE(routed_recording != nullptr);
    REQUIRE(routed_recording->settings_edits(written).empty());

    const control_parameters_window unfound("Playback", unattached(), std::weak_ptr<owned_arm>(), read_control_parameters(nothing, parameters_at), std::string(parameters_at));
    const trajectory_recording_window untaken("Recording", unattached(), std::weak_ptr<owned_arm>(), read_recording(nothing, recording_at), std::string(recording_at));

    REQUIRE(unfound.as_configurable() != nullptr);
    REQUIRE(untaken.as_configurable() != nullptr);
    REQUIRE(unfound.as_configurable()->settings_edits(nothing).empty());
    REQUIRE(untaken.as_configurable()->settings_edits(nothing).empty());

    const control_parameters_window::settings rate{0.625f};
    const control_parameters_window turned("Playback", unattached(), std::weak_ptr<owned_arm>(), rate, std::string(parameters_at));

    REQUIRE(turned.as_configurable() != nullptr);
    require_same_edits(turned.as_configurable()->settings_edits(written), config::unsaved_edits(written, write_control_parameters(rate, parameters_at)));
}

TEST_CASE("a window no key path was named for offers nothing", "[manipulator][configuration]")
{
    const control_parameters_window playback("Playback", unattached(), std::weak_ptr<owned_arm>(), control_parameters_window::settings{0.3f});

    REQUIRE(playback.settings_path().empty());
    REQUIRE(playback.as_configurable() == nullptr);
}

// The seam an application calls when it saves is the window's own `settings_edits`, so these four
// drive that rather than the free function beneath it: a window holding a state its document does
// not carry offers the difference, and the document that difference is saved into reads back as the
// state the window held.
TEST_CASE("the joint control window's own edits save and read back as the mode it holds", "[manipulator][configuration]")
{
    const std::shared_ptr<arm_publisher> published = published_arm();
    const joint_control_window::settings held{control_mode::preview};
    const joint_control_window panel("Joint control", published->reader(), std::weak_ptr<owned_arm>(), held, std::string(joint_control_at));

    const std::vector<config::edit> offered = panel.settings_edits(starter("joint-control-window.xml"));

    REQUIRE_FALSE(offered.empty());
    REQUIRE(read_joint_control(saved_and_reloaded("joint-control-window.xml", offered), joint_control_at).mode == held.mode);
}

TEST_CASE("the task space window's own edits save and read back as the shape and mode it holds", "[manipulator][configuration]")
{
    const std::shared_ptr<arm_publisher> published = published_arm();
    const task_space_window::settings held{task_space_window::motion_shape::lin, control_mode::preview};
    const task_space_window panel("Task space", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), held, std::string(task_space_at));

    const std::vector<config::edit> offered = panel.settings_edits(starter("task-space-window.xml"));

    REQUIRE_FALSE(offered.empty());

    const config::document carried = saved_and_reloaded("task-space-window.xml", offered);

    REQUIRE(read_task_space(carried, task_space_at).shape == held.shape);
    REQUIRE(read_task_space(carried, task_space_at).mode == held.mode);
}

TEST_CASE("the tool jog window's own edits save and read back as the mode it holds", "[manipulator][configuration]")
{
    const std::shared_ptr<arm_publisher> published = published_arm();
    const tool_jog_window::settings held{control_mode::preview};
    const tool_jog_window panel("Tool jog", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), held, std::string(tool_jog_at));

    const std::vector<config::edit> offered = panel.settings_edits(starter("tool-jog-window.xml"));

    REQUIRE_FALSE(offered.empty());
    REQUIRE(read_tool_jog(saved_and_reloaded("tool-jog-window.xml", offered), tool_jog_at).mode == held.mode);
}

TEST_CASE("the screw jog window's own edits save and read back as the mode it holds", "[manipulator][configuration]")
{
    const std::shared_ptr<arm_publisher> published = published_arm();
    const screw_jog_window::settings held{control_mode::preview};
    const screw_jog_window panel("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), held, std::string(screw_jog_at));

    const std::vector<config::edit> offered = panel.settings_edits(starter("screw-jog-window.xml"));

    REQUIRE_FALSE(offered.empty());
    REQUIRE(read_screw_jog(saved_and_reloaded("screw-jog-window.xml", offered), screw_jog_at).mode == held.mode);
}

TEST_CASE("a window whose document already reads as its state offers no edit at all", "[manipulator][configuration]")
{
    const config::document carried                 = starter("joint-control-unmoved.xml");
    const std::shared_ptr<arm_publisher> published = published_arm();
    const joint_control_window panel("Joint control", published->reader(), std::weak_ptr<owned_arm>(), read_joint_control(carried, joint_control_at), std::string(joint_control_at));

    REQUIRE(panel.settings_edits(carried).empty());
}

// The joints a plot leaves out are the one thing it keeps, and a document naming none is what says
// every joint is drawn -- so the absent value and the empty value have to read back as the same
// thing.
TEST_CASE("the joints a curve plot leaves out save and read back as the joints it holds", "[manipulator][configuration]")
{
    const joint_curve_window::settings held{{0u, 2u}};
    const joint_curve_window panel("Joint curves", unattached(), held, std::string(joint_curves_at));
    const config::configurable *routed = panel.as_configurable();

    REQUIRE(routed != nullptr);
    REQUIRE(routed->settings_path() == joint_curves_at);

    const config::document carried = saved_and_reloaded("joint-curves-left-out.xml", routed->settings_edits(starter("joint-curves-starter.xml")));

    CHECK(read_joint_curves(carried, joint_curves_at).hidden == std::vector<std::size_t>{0u, 2u});
}

TEST_CASE("a document naming no hidden joint opens the curve plot with every joint drawn", "[manipulator][configuration]")
{
    const config::document written = starter("joint-curves-untouched.xml");
    const config::document nothing = nothing_carried("joint-curves-never-authored.xml");

    CHECK(read_joint_curves(written, joint_curves_at).hidden.empty());
    CHECK(read_joint_curves(nothing, joint_curves_at).hidden.empty());

    const joint_curve_window opened("Joint curves", unattached(), read_joint_curves(written, joint_curves_at), std::string(joint_curves_at));

    REQUIRE(opened.as_configurable() != nullptr);
    CHECK(opened.state().hidden.empty());
    CHECK(opened.as_configurable()->settings_edits(written).empty());
}

// The two ends, the three switches and the chosen shape are one group, so what a save has to carry
// for a comparison to open where it was left is asserted over all six at once.
TEST_CASE("every path comparison field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    path_comparison_window::settings written;
    written.first       = configuration(0.25, -0.5);
    written.second      = configuration(-1.25, 0.75);
    written.joint_space = false;
    written.decoupled   = true;
    written.screw       = false;
    written.played      = compared_path::decoupled;

    const path_comparison_window::settings read =
            read_path_comparison(saved_and_reloaded("comparison-round-trip.xml", write_path_comparison(written, comparison_at)), comparison_at, 2u);

    CHECK(read.first.isApprox(written.first));
    CHECK(read.second.isApprox(written.second));
    CHECK_FALSE(read.joint_space);
    CHECK(read.decoupled);
    CHECK_FALSE(read.screw);
    CHECK(read.played == compared_path::decoupled);
}

// An end of the wrong width leaves the window opening at the pair its own opening answers, and the
// end beside it is still read.
TEST_CASE("a path comparison end of the wrong width is declined and the end beside it is still read", "[manipulator][configuration]")
{
    path_comparison_window::settings written;
    written.first  = joint_vector::Constant(5, 0.25);
    written.second = configuration(-1.25, 0.75);

    const path_comparison_window::settings read =
            read_path_comparison(saved_and_reloaded("comparison-wrong-width.xml", write_path_comparison(written, comparison_at)), comparison_at, 2u);

    CHECK(read.first.size() == 0);
    CHECK(read.second.isApprox(written.second));
}

TEST_CASE("the path comparison settings read back from a starter document at every declared fallback", "[manipulator][configuration]")
{
    const path_comparison_window::settings opened;
    const path_comparison_window::settings read = read_path_comparison(starter("comparison-starter.xml"), comparison_at, 2u);

    CHECK(read.first.size() == 0);
    CHECK(read.second.size() == 0);
    CHECK(read.joint_space == opened.joint_space);
    CHECK(read.decoupled == opened.decoupled);
    CHECK(read.screw == opened.screw);
    CHECK(read.played == opened.played);
}

TEST_CASE("a tool window holding no tool opens at the loader whatever view its document names, and offers nothing until a view is named or moved", "[manipulator][configuration]")
{
    const config::document nothing = nothing_carried("tool-view-omitted.xml");
    const std::string view_key     = key_under(at, "view");

    REQUIRE(nothing.origin_of(view_key).kind == config::origin_kind::fallback);
    REQUIRE(read_tool(nothing, at).selected_view == tool_window::tool_view::load_stl);

    scheduler::scheduler loop(scheduler::inline_workers);
    standing_stencil bare = stand(loop, attached_models{});
    tool_window untouched("Tool settings", *bare.shown, bare.published->reader(), std::weak_ptr<owned_arm>(), reference, read_tool(nothing, at), std::string(at));
    untouched.initialize();

    REQUIRE(untouched.state().selected_view == tool_window::tool_view::load_stl);
    REQUIRE(untouched.settings_edits(nothing).empty());

    tool_window::settings transforming = read_tool(nothing, at);
    transforming.selected_view         = tool_window::tool_view::kinematics_transform;

    const config::document carried = saved_and_reloaded("tool-view-carried.xml", write_tool(transforming, at));
    standing_stencil written       = stand(loop, attached_models{});
    tool_window recorded("Tool settings", *written.shown, written.published->reader(), std::weak_ptr<owned_arm>(), reference, read_tool(carried, at), std::string(at));
    recorded.initialize();

    REQUIRE(carried.origin_of(view_key).kind == config::origin_kind::source);
    REQUIRE(read_tool(carried, at).selected_view == tool_window::tool_view::kinematics_transform);
    REQUIRE(recorded.state().selected_view == tool_window::tool_view::load_stl);
    REQUIRE(recorded.settings_edits(carried).empty());

    tool_window::settings composed = read_tool(nothing, at);
    composed.selected_view         = tool_window::tool_view::graphics_transform;

    standing_stencil named = stand(loop, attached_models{});
    tool_window opened("Tool settings", *named.shown, named.published->reader(), std::weak_ptr<owned_arm>(), reference, composed, std::string(at));
    opened.initialize();

    REQUIRE(opened.state().selected_view == tool_window::tool_view::load_stl);
    require_one_edit(opened.settings_edits(nothing), view_key, "graphics_transform");

    standing_stencil turned = stand(loop, attached_models{});
    tool_window moved("Tool settings", *turned.shown, turned.published->reader(), std::weak_ptr<owned_arm>(), reference, read_tool(nothing, at), std::string(at));
    moved.initialize();

    // A combo opens its keyboard cursor on the entry it is showing and wraps, so the steps are
    // counted from the loader this window opened at rather than from the list's first entry.
    {
        tests::imgui_frame frames;
        const drawing draw = [&moved] { moved.render(); };
        take_entry_on(frames, draw, "Tool settings", "Tool view", 2u);
        REQUIRE(frames.has_draw_data());
    }

    REQUIRE(moved.state().selected_view == tool_window::tool_view::graphics_transform);
    require_one_edit(moved.settings_edits(nothing), view_key, "graphics_transform");
}

TEST_CASE("a world object window opens where its initializer puts it and offers nothing over the document it was read from, until its activation is pressed",
          "[manipulator][configuration]")
{
    const config::document nothing = nothing_carried("world-view-omitted.xml");
    const std::string view_key     = key_under(world_at, "view");
    const std::string active_key   = key_under(world_at, "active");

    REQUIRE(nothing.origin_of(view_key).kind == config::origin_kind::fallback);
    REQUIRE(read_world_object(nothing, world_at).selected_view == world_object_window::world_view::load_stl);
    REQUIRE_FALSE(read_world_object(nothing, world_at).active);

    world_object_window::settings standing = read_world_object(nothing, world_at);
    standing.active                        = true;
    standing.selected_view                 = world_object_window::world_view::transform;

    const config::document carried = saved_and_reloaded("world-view-carried.xml", write_world_object(standing, world_at));

    REQUIRE(carried.origin_of(view_key).kind == config::origin_kind::source);
    REQUIRE(read_world_object(carried, world_at).selected_view == world_object_window::world_view::transform);
    REQUIRE(read_world_object(carried, world_at).active);

    scheduler::scheduler loop(scheduler::inline_workers);
    standing_stencil bare = stand(loop, attached_models{});
    world_object_window loose("World object", *bare.shown, reference, read_world_object(carried, world_at), std::string(world_at));
    loose.initialize();

    REQUIRE(loose.state().selected_view == world_object_window::world_view::load_stl);
    REQUIRE(loose.settings_edits(carried).empty());

    standing_stencil seated = stand(loop, attached_models{nullptr, a_mesh()});
    world_object_window held("World object", *seated.shown, reference, read_world_object(carried, world_at), std::string(world_at));
    held.initialize();

    REQUIRE(held.state().selected_view == world_object_window::world_view::transform);
    REQUIRE(held.settings_edits(carried).empty());

    {
        tests::imgui_frame frames;
        const drawing draw = [&held] { held.render(); };
        press_on(frames, draw, "World object", "Active");
        REQUIRE(frames.has_draw_data());
    }

    const std::vector<config::edit> offered = held.settings_edits(carried);

    REQUIRE(held.state().selected_view == world_object_window::world_view::load_stl);
    REQUIRE(offered.size() == 2u);
    REQUIRE(offered.front().key == active_key);
    REQUIRE(offered.front().value == "false");
    REQUIRE(offered.back().key == view_key);
    REQUIRE(offered.back().value == "load_stl");
}
