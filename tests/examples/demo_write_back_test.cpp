#include "scratch_documents.h"

#include "demo_documents.h"
#include "demo_write_back.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace scratch_documents;

namespace {

constexpr const char *label = "view/label";

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

// One window's worth of state, standing for whatever a person moved: it offers a single value and
// narrows that offer to nothing once the document already reads as it.
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

// What a composition hands over when it is saved, which is the shown windows in the order they
// stand in.
struct composition
{
    explicit composition(const scratch &where, const std::filesystem::path &named = "machine.xml")
            : mine(where.seeds(), where.state())
            , bound{machine_shape(), mine.reading(named), config::expectation::partial}
            , preferences{preferences_shape(), config::resolve("praxis-preferences.xml", where.state()), config::expectation::partial}
            , writing(bound, config::load_or_defaults(bound).values, preferences, config::load_or_defaults(preferences).values, mine)
    {
    }

    demo::documents mine;
    config::binding bound;
    config::binding preferences;
    demo::write_back writing;
};

void save_through(demo::write_back &writing, const one_window &shown)
{
    const std::array<const config::configurable *, 1> panel{&shown};
    writing.save(panel);
}

std::string label_at(const std::filesystem::path &document)
{
    const config::outcome read                     = config::load_or_defaults(machine_shape(), config::location{document, document}, config::expectation::partial);
    const expected<std::string, config::error> was = read.values.text(label);

    return was ? was.value() : std::string();
}

}

TEST_CASE("A save leaves the shipped document byte for byte as it was", "[examples][write-back]")
{
    const scratch where("saving-never-writes-the-seed");
    author(where.seeds() / "machine.xml", authored);

    composition composed(where);
    save_through(composed.writing, one_window("what a person typed"));

    REQUIRE(bytes_of(where.seeds() / "machine.xml") == std::string(authored));
    REQUIRE(files_in(where.seeds()) == 1u);
}

TEST_CASE("A save lands in this application's own copy", "[examples][write-back]")
{
    const scratch where("saving-lands-in-the-copy");
    author(where.seeds() / "machine.xml", authored);

    composition composed(where);
    save_through(composed.writing, one_window("what a person typed"));

    REQUIRE(std::filesystem::exists(where.state() / "machine.xml"));
    REQUIRE(label_at(where.state() / "machine.xml") == "what a person typed");
    REQUIRE(label_at(where.seeds() / "machine.xml") == "as it ships");
}

TEST_CASE("The first copy is the seed rather than a document written from the declaration", "[examples][write-back]")
{
    const scratch where("the-copy-keeps-what-was-authored");
    author(where.seeds() / "machine.xml", authored);

    composition composed(where);
    save_through(composed.writing, one_window("what a person typed"));

    REQUIRE(bytes_of(where.state() / "machine.xml").find("<!-- the line an author wrote -->") != std::string::npos);
}

TEST_CASE("A second save reaches the same copy and still spares the seed", "[examples][write-back]")
{
    const scratch where("saving-twice");
    author(where.seeds() / "machine.xml", authored);

    composition composed(where);
    save_through(composed.writing, one_window("what a person typed"));
    save_through(composed.writing, one_window("what they typed after that"));

    REQUIRE(label_at(where.state() / "machine.xml") == "what they typed after that");
    REQUIRE(bytes_of(where.seeds() / "machine.xml") == std::string(authored));
    REQUIRE(files_in(where.seeds()) == 1u);
}

TEST_CASE("A composition rebound to a document with no seed writes only under the state directory", "[examples][write-back]")
{
    const scratch where("a-name-no-seed-carries");
    author(where.seeds() / "machine.xml", authored);

    composition composed(where);
    const config::binding supplied{machine_shape(), composed.mine.reading("supplied-chain.xml"), config::expectation::partial};
    composed.writing.composing(supplied, config::load_or_defaults(supplied).values);
    save_through(composed.writing, one_window("a chain typed by hand"));

    REQUIRE(label_at(where.state() / "supplied-chain.xml") == "a chain typed by hand");
    REQUIRE(files_in(where.seeds()) == 1u);
}

TEST_CASE("A save of a document named under a directory keeps that directory", "[examples][write-back]")
{
    const scratch where("a-name-carrying-a-directory");
    author(where.seeds() / "beside" / "machine.xml", authored);

    composition composed(where, "beside/machine.xml");
    save_through(composed.writing, one_window("what a person typed"));

    REQUIRE(label_at(where.state() / "beside" / "machine.xml") == "what a person typed");
    REQUIRE_FALSE(std::filesystem::exists(where.state() / "machine.xml"));
    REQUIRE(bytes_of(where.seeds() / "beside" / "machine.xml") == std::string(authored));
}

TEST_CASE("The leaving answer keeps its three options", "[examples][write-back]")
{
    const config::declaration shape              = preferences_shape();
    const std::span<const config::node> declared = shape.nodes();
    const auto found                             = std::ranges::find_if(declared, [](const config::node &one) { return one.path == "editing/on_leaving"; });

    REQUIRE(found != declared.end());
    REQUIRE(found->kind == config::field_kind::choice);
    REQUIRE(found->allowed == std::vector<std::string>{"ask", "keep", "discard"});
    REQUIRE(found->fallback == "ask");
}
