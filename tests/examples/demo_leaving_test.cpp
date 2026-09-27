#include "scratch_documents.h"

#include "demo_documents.h"
#include "demo_write_back.h"

#include "praxis/scene/composition.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <string>
#include <vector>
#include <utility>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace scratch_documents;

namespace {

constexpr const char *label      = "view/label";
constexpr const char *unwritable = "the offer names a place the document cannot hold";

config::declaration machine_shape()
{
    config::declaration shape("machine");
    shape.group("view");
    shape.field(label, config::field_kind::text, "");

    return shape;
}

config::declaration preferences_shape()
{
    config::declaration shape("preferences");
    demo::declare_leaving(shape);

    return shape;
}

config::binding preferences_in(const scratch &where)
{
    return config::binding{preferences_shape(), config::resolve("praxis-preferences.xml", where.state()), config::expectation::partial};
}

// A window whose offer cannot be written, whatever the document carries.
class refused_window : public config::configurable
{
public:
    std::string_view settings_path() const override
    {
        return "view";
    }

    std::vector<config::edit> settings_edits(const config::document &) const override
    {
        return {config::edit{"view", unwritable, config::edit_kind::refused}};
    }
};

// A window offering a single value, narrowed to nothing once the document already reads as it.
class one_window : public config::configurable
{
public:
    explicit one_window(std::string typed)
            : m_typed(std::move(typed))
    {
    }

    std::string_view settings_path() const override
    {
        return "view";
    }

    std::vector<config::edit> settings_edits(const config::document &carried) const override
    {
        const std::array<config::edit, 1> offered{config::edit{label, m_typed}};

        return config::unsaved_edits(carried, offered);
    }

private:
    std::string m_typed;
};

struct holder
{
    explicit holder(const scratch &where)
            : mine(where.seeds(), where.state())
            , bound{machine_shape(), mine.reading("machine.xml"), config::expectation::partial}
            , preferences(preferences_in(where))
            , writing(bound, config::load_or_defaults(bound).values, preferences, config::load_or_defaults(preferences).values, mine)
    {
    }

    demo::documents mine;
    config::binding bound;
    config::binding preferences;
    demo::write_back writing;
};

std::string label_at(const std::filesystem::path &document)
{
    return config::load_or_defaults(machine_shape(), config::location{document, document}, config::expectation::partial).values.text(label).value_or(std::string());
}

}

TEST_CASE("An offer that cannot be written is left to decide, and keeping it fails in its words until it is discarded", "[examples][write-back]")
{
    const scratch where("keeping-a-refused-offer");
    author(where.seeds() / "machine.xml", authored);

    holder held(where);
    const refused_window shown;
    const std::array<const config::configurable *, 1> panel{&shown};

    CHECK(held.writing.anything_to_decide(panel));

    const expected<void, std::string> kept = held.writing.resolve(scene::leaving_answer{true, true}, panel);
    REQUIRE_FALSE(kept.has_value());
    CHECK(kept.error() == unwritable);
    CHECK_FALSE(std::filesystem::exists(where.state() / "machine.xml"));
    CHECK_FALSE(std::filesystem::exists(where.state() / "praxis-preferences.xml"));
    CHECK(held.writing.anything_to_decide(panel));

    CHECK(held.writing.resolve(scene::leaving_answer{false, false}, panel).has_value());
    CHECK_FALSE(std::filesystem::exists(where.state() / "machine.xml"));
}

TEST_CASE("A remembered keep that cannot be written puts the question, and one that can is carried out without it", "[examples][write-back]")
{
    const scratch where("a-remembered-keep");
    author(where.seeds() / "machine.xml", authored);
    std::filesystem::create_directories(where.state());

    const std::array<config::edit, 1> remembered{config::edit{"editing/on_leaving", "keep"}};
    REQUIRE(config::save(preferences_in(where), remembered).has_value());

    holder held(where);

    const refused_window refused;
    const std::array<const config::configurable *, 1> cannot{&refused};
    CHECK(held.writing.anything_to_decide(cannot));
    CHECK_FALSE(std::filesystem::exists(where.state() / "machine.xml"));

    const one_window typed("what a person typed");
    const std::array<const config::configurable *, 1> can{&typed};
    CHECK_FALSE(held.writing.anything_to_decide(can));
    CHECK(label_at(where.state() / "machine.xml") == "what a person typed");
}
