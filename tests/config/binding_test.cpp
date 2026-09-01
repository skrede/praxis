#include "captured_log.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <utility>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

constexpr std::string_view hand_written = "<probe>\n"
                                          "    <!-- The panel as it was left. -->\n"
                                          "    <panel scale=\"1.5\">\n"
                                          "        <title>Overview</title>\n"
                                          "    </panel>\n"
                                          "</probe>\n";

declaration described()
{
    declaration shape("probe");
    shape.group("panel").field("panel/scale", field_kind::real, "1.0").field("panel/title", field_kind::text, "untitled");
    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-binding";
    std::filesystem::create_directories(directory);
    return directory;
}

std::filesystem::path authored(const std::string &name, std::string_view text)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << text;
    return where;
}

std::string text_of(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    std::ostringstream all;
    all << in.rdbuf();
    return all.str();
}

binding bound_to(const std::filesystem::path &where, expectation carries)
{
    return binding{described(), resolve(where, where.parent_path()), carries};
}

// A stand-in for a settings-carrying window: built from the values a document answered, and
// answering afterwards for the path those values live under and for the edits its state stands for.
class panel : public configurable
{
public:
    panel(std::string at, const document &carried)
            : m_at(std::move(at))
            , m_scale(carried.real(m_at + "/scale").value_or(0.0))
            , m_title(carried.text(m_at + "/title").value_or(std::string()))
    {
    }

    double scale() const
    {
        return m_scale;
    }

    void set_scale(double chosen)
    {
        m_scale = chosen;
    }

    std::string_view settings_path() const override
    {
        return m_at;
    }

    std::vector<edit> settings_edits(const document &carried) const override
    {
        const std::vector<edit> stands{edit{m_at + "/scale", exact_text(m_scale)}, edit{m_at + "/title", m_title}};

        return unsaved_edits(carried, stands);
    }

private:
    std::string m_at;
    double m_scale;
    std::string m_title;
};

std::vector<edit> edits_of(const panel &shown, const document &carried)
{
    const configurable *const one = &shown;
    return shown_edits(std::span<const configurable *const>(&one, 1), carried);
}

}

TEST_CASE("a composition built from its own document writes nothing while it is left untouched, and a saved value survives a reload", "[config]")
{
    const std::filesystem::path where = authored("panel.xml", hand_written);
    const binding bound               = bound_to(where, expectation::complete);

    const document carried = load_or_defaults(bound).values;
    panel shown("panel", carried);
    REQUIRE(edits_of(shown, carried).empty());

    const std::string untouched = text_of(where);
    REQUIRE(save(bound, edits_of(shown, carried)).has_value());
    REQUIRE(text_of(where) == untouched);

    const double chosen = shown.scale() + 1.0;
    shown.set_scale(chosen);

    const std::vector<edit> offered = edits_of(shown, carried);
    REQUIRE(offered.size() == 1u);
    REQUIRE(offered.front().key == "panel/scale");
    REQUIRE(save(bound, offered).has_value());

    const std::string was      = "\"" + exact_text(carried.real("panel/scale").value()) + "\"";
    std::string only_the_scale = untouched;
    only_the_scale.replace(only_the_scale.find(was), was.size(), "\"" + exact_text(chosen) + "\"");
    REQUIRE(text_of(where) == only_the_scale);

    const document reread = load_or_defaults(bound).values;
    const panel again("panel", reread);
    REQUIRE(again.scale() == chosen);
    REQUIRE(edits_of(again, reread).empty());
}

TEST_CASE("a value saved through one binding leaves every other document byte for byte as it was", "[config]")
{
    const std::filesystem::path mine   = authored("mine.xml", hand_written);
    const std::filesystem::path theirs = authored("theirs.xml", hand_written);
    const binding ours                 = bound_to(mine, expectation::complete);
    const binding others               = bound_to(theirs, expectation::complete);

    const document carried = load_or_defaults(ours).values;
    panel shown("panel", carried);
    shown.set_scale(shown.scale() + 1.0);

    const std::string untouched = text_of(theirs);
    REQUIRE(save(ours, edits_of(shown, carried)).has_value());
    REQUIRE(text_of(mine) != untouched);
    REQUIRE(text_of(theirs) == untouched);

    const document theirs_still = load_or_defaults(others).values;
    const panel unaffected("panel", theirs_still);
    REQUIRE(unaffected.scale() == carried.real("panel/scale").value());
    REQUIRE(edits_of(unaffected, theirs_still).empty());
}

TEST_CASE("a document a caller declared partial reports its absence out of the operator's way, and one declared complete still reports it", "[config]")
{
    const std::filesystem::path nowhere = scratch() / "not-written.xml";
    std::filesystem::remove(nowhere);

    const std::string quiet = praxis::tests::reported_by([&] { static_cast<void>(load_or_defaults(bound_to(nowhere, expectation::partial))); });
    REQUIRE(quiet.find("there is no configuration file at") == std::string::npos);
    REQUIRE(quiet.find("warning") == std::string::npos);

    const std::string spoken = praxis::tests::reported_by([&] { static_cast<void>(load_or_defaults(bound_to(nowhere, expectation::complete))); });
    REQUIRE(spoken.find("there is no configuration file at") != std::string::npos);
    REQUIRE(spoken.find("warning") != std::string::npos);
}

TEST_CASE("a composition built from a document carrying none of its values offers nothing to write", "[config]")
{
    const std::filesystem::path nowhere = scratch() / "never-written.xml";
    std::filesystem::remove(nowhere);

    const document carried = load_or_defaults(bound_to(nowhere, expectation::partial)).values;
    REQUIRE(carried.origin_of("panel/scale").kind == origin_kind::fallback);
    REQUIRE(carried.origin_of("panel/title").kind == origin_kind::fallback);

    const panel shown("panel", carried);
    REQUIRE(edits_of(shown, carried).empty());
}
