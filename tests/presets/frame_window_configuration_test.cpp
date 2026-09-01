#include "scratch_directory.h"

#include "praxis/presets/euler_rungs.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_window.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/configuration.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/strand.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include <Eigen/Core>

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <utility>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

constexpr std::string_view at = "arrangement";

const std::vector<std::string> &objects()
{
    static const std::vector<std::string> named{"base", "upper", "tip"};

    return named;
}

const std::filesystem::path &scratch()
{
    return fixture::shared_scratch_directory();
}

config::declaration described()
{
    config::declaration shape("probe");
    declare_arrangement(shape, at);

    return shape;
}

// A starter document names no instance of a collection, so the arrangement is authored with one
// instance per object: an object the document carries no instance for contributes no edit at all.
config::document authored()
{
    const std::filesystem::path where = scratch() / "arranged.xml";
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><arrangement>";
    for(const std::string &named : objects())
        out << "<object name=\"" << named << "\" euler_order=\"ZYX\"><position x=\"0\" y=\"0\" z=\"0\"/><euler x=\"0\" y=\"0\" z=\"0\"/></object>";
    out << "</arrangement></probe>\n";
    out.close();

    const expected<config::document, config::error> read = config::load(described(), config::resolve(where, scratch()));
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());

    return read.value();
}

std::vector<stencil_object> described_objects()
{
    std::vector<stencil_object> shapes;
    for(const std::string &named : objects())
        shapes.push_back(stencil_object{named, axes_settings{}, object_body{body_shape::none, 0.0, nullptr}});

    return shapes;
}

frame_window::placement placed(const Eigen::Vector3f &position, std::optional<std::size_t> parent)
{
    frame_window::placement one;
    one.position = position;
    one.parent   = parent;

    return one;
}

frame_window::settings arranged()
{
    frame_window::settings chosen;
    chosen.objects = {placed({1.f, 2.f, 3.f}, std::nullopt), placed({4.f, 5.f, 6.f}, std::size_t{0}), placed({7.f, 8.f, 9.f}, std::size_t{1})};

    return chosen;
}

std::string object_element(std::string_view name, std::string_view parent, float along)
{
    return "<object name=\"" + std::string(name) + "\" parent=\"" + std::string(parent) + "\" euler_order=\"ZYX\"><position x=\"" + std::to_string(along) +
            "\" y=\"0\" z=\"0\"/><euler x=\"0\" y=\"0\" z=\"0\"/></object>";
}

config::binding document_at(const std::string &name, const std::string &body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><arrangement>" << body << "</arrangement></probe>\n";
    out.close();

    return config::binding{described(), config::resolve(where, scratch()), config::expectation::partial};
}

scene::preset_site unwired(threepp::Scene &target)
{
    const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};

    return scene::preset_site{target, scheduler::strand{}, scheduler::strand{}, [] {}, nowhere, nowhere, {}};
}

// Whether there is still something to save, asked of one implementor the way a composition asks it
// of everything it shows.
bool still_to_save(const config::configurable &one, const config::document &carried)
{
    const config::configurable *const shown = &one;

    return config::anything_unsaved(std::span<const config::configurable *const>(&shown, 1), carried);
}

// A window that has already decided what moved: it offers the edits it was handed, which is what an
// arrangement's own mapping mints for the placements the window shows.
class decided : public config::configurable
{
public:
    decided(std::string where, std::vector<config::edit> offered)
            : m_at(std::move(where))
            , m_offered(std::move(offered))
    {
    }

    std::string_view settings_path() const override
    {
        return m_at;
    }

    std::vector<config::edit> settings_edits(const config::document &) const override
    {
        return m_offered;
    }

private:
    std::string m_at;
    std::vector<config::edit> m_offered;
};

void require_one_offered(const config::document &carried, const frame_window::settings &opened, const frame_window::settings &moved, const std::string &key, const std::string &value)
{
    const std::vector<config::edit> offered = write_arrangement(carried, opened, moved, at, objects());

    REQUIRE(offered.size() == 1u);
    INFO(offered.front().key);
    REQUIRE(offered.front().key == key);
    REQUIRE(offered.front().value == value);

    const decided window(std::string(at), offered);
    REQUIRE(still_to_save(window, carried));
}

// The arrangement panel is the window a rung composes behind its selector; the selector before it and
// the readouts after it read the stencil rather than the document and say nothing about what opened.
std::shared_ptr<frame_window> arrangement_panel(const std::shared_ptr<scene::preset> &composed)
{
    REQUIRE(composed != nullptr);
    REQUIRE(composed->windows.size() > 1u);

    const std::shared_ptr<frame_window> panel = std::dynamic_pointer_cast<frame_window>(composed->windows[1]);
    REQUIRE(panel != nullptr);

    return panel;
}

std::shared_ptr<scene::preset> rung(threepp::Scene &target, presets::euler_rung which, const config::document &carried)
{
    return presets::euler_rung_preset(unwired(target), baseline(), which, presets::arrangement_source{std::string(at), carried});
}

}

TEST_CASE("the arrangement window answers with the edits the arrangement mapping writes", "[rigid_motion][configuration]")
{
    const config::document carried = authored();

    threepp::Scene target;
    frame_stencil body(target, described_objects(), frame_ops{});
    const frame_window panel("Frame", body, frame_ops{}, arranged(), std::string(at));

    const config::configurable *routed = panel.as_configurable();
    REQUIRE(routed != nullptr);
    REQUIRE(routed->settings_path() == at);

    const std::vector<config::edit> answered = routed->settings_edits(carried);
    const std::vector<config::edit> written  = write_arrangement(carried, arranged(), arranged(), at, objects());

    REQUIRE_FALSE(written.empty());
    REQUIRE(answered.size() == written.size());
    for(std::size_t which = 0u; which < written.size(); ++which)
    {
        INFO(written[which].key);
        REQUIRE(answered[which].key == written[which].key);
        REQUIRE(answered[which].value == written[which].value);
    }
}

TEST_CASE("an arrangement window no key path was named for offers nothing", "[rigid_motion][configuration]")
{
    threepp::Scene target;
    frame_stencil body(target, described_objects(), frame_ops{});
    const frame_window panel("Frame", body, frame_ops{}, arranged());

    REQUIRE(panel.settings_path().empty());
    REQUIRE(panel.as_configurable() == nullptr);
}

TEST_CASE("a preset composed from a document opens at what it carries", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("opens-at.xml", object_element("Frame", "", 1.5f) + object_element("Second", "Frame", 2.5f));
    const config::outcome carried = config::load_or_defaults(bound);

    threepp::Scene target;
    const std::shared_ptr<frame_window> panel = arrangement_panel(rung(target, presets::euler_rung::paired_frames, carried.values));

    REQUIRE(panel->settings_path() == at);
    REQUIRE(panel->as_configurable() != nullptr);
    REQUIRE(panel->state().objects.front().position.x() == 1.5f);
    REQUIRE(panel->state().objects.back().position.x() == 2.5f);
    REQUIRE(panel->state().objects.back().parent == std::optional<std::size_t>(std::size_t{0}));
}

TEST_CASE("a composition built from a document does not differ from it", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("untouched.xml", object_element("Frame", "", 1.5f) + object_element("Second", "Frame", 2.5f));
    const config::outcome carried = config::load_or_defaults(bound);

    threepp::Scene target;
    const std::shared_ptr<frame_window> panel = arrangement_panel(rung(target, presets::euler_rung::paired_frames, carried.values));
    const config::configurable *routed        = panel->as_configurable();
    REQUIRE(routed != nullptr);

    REQUIRE_FALSE(still_to_save(*routed, carried.values));
}

TEST_CASE("a composition nobody touched against a document missing an object offers nothing to write", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("untouched-missing.xml", object_element("Frame", "", 1.5f));
    const config::outcome carried = config::load_or_defaults(bound);

    threepp::Scene target;
    const std::shared_ptr<frame_window> panel = arrangement_panel(rung(target, presets::euler_rung::paired_frames, carried.values));
    const config::configurable *routed        = panel->as_configurable();
    REQUIRE(routed != nullptr);

    REQUIRE(routed->settings_edits(carried.values).empty());
    REQUIRE_FALSE(still_to_save(*routed, carried.values));
}

TEST_CASE("a composition built against a document naming no object at all offers nothing to write", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("no-objects.xml", std::string());
    const config::outcome carried = config::load_or_defaults(bound);

    threepp::Scene target;
    const std::shared_ptr<frame_window> panel = arrangement_panel(rung(target, presets::euler_rung::single_frame, carried.values));
    const config::configurable *routed        = panel->as_configurable();
    REQUIRE(routed != nullptr);

    REQUIRE(routed->settings_edits(carried.values).empty());
    REQUIRE_FALSE(still_to_save(*routed, carried.values));
}

TEST_CASE("a composition built against an instance carrying only some of its fields offers nothing to write", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("some-fields.xml", "<object name=\"Frame\"><position x=\"1.5\"/></object>");
    const config::outcome carried = config::load_or_defaults(bound);

    threepp::Scene target;
    const std::shared_ptr<frame_window> panel = arrangement_panel(rung(target, presets::euler_rung::single_frame, carried.values));
    const config::configurable *routed        = panel->as_configurable();
    REQUIRE(routed != nullptr);

    REQUIRE(panel->state().objects.back().position.x() == 1.5f);
    REQUIRE(panel->state().objects.back().position.z() > 0.f);

    REQUIRE(routed->settings_edits(carried.values).empty());
    REQUIRE_FALSE(still_to_save(*routed, carried.values));
}

TEST_CASE("an object the document names no instance of keeps the placement the preset supplies", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("partial.xml", object_element("Second", "", 2.5f));
    const config::outcome carried = config::load_or_defaults(bound);

    threepp::Scene target;
    const std::shared_ptr<frame_window> panel = arrangement_panel(rung(target, presets::euler_rung::paired_frames, carried.values));

    REQUIRE(panel->state().objects[0].position.x() != 0.f);
    REQUIRE(panel->state().objects[0].position.z() > 0.f);
    REQUIRE(panel->state().objects[1].position.x() == 2.5f);
}

// Neither rung names a parent for any of its placements, so what a document leaves out is read
// against an arrangement written here instead: the property is the mapping's, not the scenario's.
TEST_CASE("an instance the document names no parent for keeps the parent the caller supplied, and one it names as nothing is expressed in the world frame",
          "[rigid_motion][configuration]")
{
    const config::binding bound                                = document_at("no-parent.xml", "<object name=\"upper\"><position x=\"2.5\" y=\"0\" z=\"0\"/></object>");
    const expected<frame_window::settings, config::error> kept = read_arrangement(config::load_or_defaults(bound).values, at, objects(), arranged());
    INFO((kept ? std::string() : kept.error().message));
    REQUIRE(kept.has_value());

    REQUIRE(kept.value().objects[1].position.x() == 2.5f);
    REQUIRE(kept.value().objects[1].parent == std::optional<std::size_t>(std::size_t{0}));

    const config::binding emptied                               = document_at("empty-parent.xml", object_element("upper", "", 2.5f));
    const expected<frame_window::settings, config::error> under = read_arrangement(config::load_or_defaults(emptied).values, at, objects(), arranged());
    REQUIRE(under.has_value());

    REQUIRE_FALSE(under.value().objects[1].parent.has_value());
}

TEST_CASE("a document naming no object of the arrangement is refused and the preset opens at its own", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("refused.xml", object_element("Frame", "bench", 1.5f));
    const config::outcome carried = config::load_or_defaults(bound);

    threepp::Scene target;
    const std::shared_ptr<frame_window> panel = arrangement_panel(rung(target, presets::euler_rung::single_frame, carried.values));

    REQUIRE(panel->state().objects.size() == 1u);
    REQUIRE(panel->state().objects.front().position.x() != 1.5f);
    REQUIRE(panel->state().objects.front().position.z() > 0.f);
}

TEST_CASE("a preset composed with no arrangement opens at its own and offers nothing", "[rigid_motion][configuration]")
{
    threepp::Scene target;

    const std::shared_ptr<frame_window> panel = arrangement_panel(presets::euler_rung_preset(unwired(target), baseline(), presets::euler_rung::single_frame));

    REQUIRE(panel->settings_path().empty());
    REQUIRE(panel->as_configurable() == nullptr);
    REQUIRE(panel->state().objects.size() == 1u);
    REQUIRE(panel->state().objects.front().position.z() > 0.f);
}

TEST_CASE("only the leaves a placement moved on are offered, with the identity that names an instance the document does not carry", "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("moved.xml", object_element("upper", "", 2.5f));
    const config::outcome carried = config::load_or_defaults(bound);

    frame_window::settings opening;
    opening.objects = {placed({1.f, 0.f, 0.f}, std::nullopt), placed({0.f, 2.f, 0.f}, std::nullopt), placed({0.f, 0.f, 3.f}, std::nullopt)};

    const expected<frame_window::settings, config::error> opened = read_arrangement(carried.values, at, objects(), opening);
    INFO((opened ? std::string() : opened.error().message));
    REQUIRE(opened.has_value());

    frame_window::settings moved = opened.value();
    moved.objects[2].position.x() += 0.5f;

    const std::vector<config::edit> changes = write_arrangement(carried.values, opened.value(), moved, at, objects());

    const std::vector<std::string> wanted{"/object[1]/name", "/object[1]/position/x"};
    REQUIRE(changes.size() == wanted.size());
    for(std::size_t which = 0u; which < wanted.size(); ++which)
    {
        INFO(changes[which].key);
        REQUIRE(changes[which].key == std::string(at) + wanted[which]);
    }
    REQUIRE(changes.front().value == "tip");
    REQUIRE(changes.back().value == "0.5");

    frame_window::settings turned = opened.value();
    turned.objects[1].euler_degrees.z() += 15.f;

    const std::vector<config::edit> about = write_arrangement(carried.values, opened.value(), turned, at, objects());

    REQUIRE(about.size() == 1u);
    INFO(about.front().key);
    REQUIRE(about.front().key == std::string(at) + "/object[0]/euler/z");
    REQUIRE(about.front().value == "15");
}

TEST_CASE("a leaf moved to the value the declaration falls back to, over an instance the document carries only part of, is offered and still has to be saved",
          "[rigid_motion][configuration]")
{
    const config::binding bound   = document_at("reset-to-zero.xml", "<object name=\"upper\"><position x=\"0.2\"/></object>");
    const config::outcome carried = config::load_or_defaults(bound);

    frame_window::settings opening;
    opening.objects                      = {placed({1.f, 0.f, 0.f}, std::nullopt), placed({0.f, 2.f, 0.f}, std::size_t{0}), placed({0.f, 0.f, 3.f}, std::nullopt)};
    opening.objects[1].euler_degrees.z() = 45.f;

    const expected<frame_window::settings, config::error> opened = read_arrangement(carried.values, at, objects(), opening);
    INFO((opened ? std::string() : opened.error().message));
    REQUIRE(opened.has_value());

    const std::string instance = std::string(at) + "/object[0]";
    REQUIRE(carried.values.origin_of(instance + "/euler/z").kind == config::origin_kind::fallback);
    REQUIRE(carried.values.origin_of(instance + "/position/y").kind == config::origin_kind::fallback);
    REQUIRE(carried.values.origin_of(instance + "/parent").kind == config::origin_kind::fallback);

    // The document reads that leaf as exactly the value the operator moved it to, so the document is
    // no reference for a window whose reading of an absent leaf is not the document's own.
    const std::vector<config::edit> reset{config::edit{instance + "/euler/z", "0"}};
    REQUIRE(config::unsaved_edits(carried.values, reset).empty());

    frame_window::settings turned       = opened.value();
    turned.objects[1].euler_degrees.z() = 0.f;
    require_one_offered(carried.values, opened.value(), turned, instance + "/euler/z", "0");

    frame_window::settings freed = opened.value();
    freed.objects[1].parent      = std::nullopt;
    require_one_offered(carried.values, opened.value(), freed, instance + "/parent", std::string());

    frame_window::settings lowered  = opened.value();
    lowered.objects[1].position.y() = 0.f;
    require_one_offered(carried.values, opened.value(), lowered, instance + "/position/y", "0");

    // The control: a value the declaration does not fall back to, offered through the same path.
    frame_window::settings raised  = opened.value();
    raised.objects[1].position.y() = 0.75f;
    require_one_offered(carried.values, opened.value(), raised, instance + "/position/y", "0.75");
}
