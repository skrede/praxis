#include "captured_log.h"

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
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <utility>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

constexpr std::string_view hand_written = "<probe>\n"
                                          "    <panel scale=\"1.5\" label=\"x\"/>\n"
                                          "</probe>\n";

constexpr std::string_view written_as_zero = "<probe>\n"
                                             "    <panel scale=\"0\" label=\"x\"/>\n"
                                             "</probe>\n";

binding authored(const std::filesystem::path &where)
{
    std::filesystem::create_directories(where.parent_path());
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << hand_written;

    declaration shape("probe");
    shape.group("panel").field("panel/scale", field_kind::real, "1.0").field("panel/label", field_kind::text, "");
    return binding{std::move(shape), resolve(where, where.parent_path()), expectation::complete};
}

std::filesystem::path scratch(const std::string &name)
{
    return std::filesystem::temp_directory_path() / "praxis-config-negative-zero" / name;
}

std::string text_of(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    std::ostringstream all;
    all << in.rdbuf();
    return all.str();
}

// An implementor offering a fixed list, whatever the document carries.
class offering : public configurable
{
public:
    explicit offering(std::vector<edit> offered)
            : m_offered(std::move(offered))
    {
    }

    std::string_view settings_path() const override
    {
        return "panel";
    }

    std::vector<edit> settings_edits(const document &) const override
    {
        return m_offered;
    }

protected:
    std::vector<edit> m_offered;
};

// An implementor narrowing its fixed list to what the document does not already read as.
class narrowing : public offering
{
public:
    using offering::offering;

    std::vector<edit> settings_edits(const document &carried) const override
    {
        return unsaved_edits(carried, m_offered);
    }
};

}

TEST_CASE("an edited -0 against a document carrying 1.5 is written as 0, whether gathered or saved directly", "[config]")
{
    const std::filesystem::path where = scratch("gathered.xml");
    const binding bound               = authored(where);
    const narrowing window({edit{"panel/scale", "-0"}});
    const std::vector<edit> offered = shown_edits(std::array<const configurable *, 1>{&window}, load_or_defaults(bound).values);
    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().kind == edit_kind::bound);
    CHECK(offered.front().value == "-0");

    REQUIRE(save(bound, offered).has_value());
    CHECK(text_of(where) == written_as_zero);
    const double reloaded = load_or_defaults(bound).values.real("panel/scale").value_or(-1.0);
    CHECK(reloaded == 0.0);
    CHECK_FALSE(std::signbit(reloaded));

    const std::filesystem::path direct_where = scratch("direct.xml");
    const binding direct                     = authored(direct_where);
    REQUIRE(save(direct, std::vector<edit>{edit{"panel/scale", "-0.0"}}).has_value());
    CHECK(text_of(direct_where) == written_as_zero);
}

TEST_CASE("a save writes every other offered text as offered, a positive zero and a text key's -0 included", "[config]")
{
    const std::filesystem::path where = scratch("verbatim.xml");
    const binding bound               = authored(where);
    REQUIRE(save(bound, std::vector<edit>{edit{"panel/scale", "0.0"}, edit{"panel/label", "-0"}}).has_value());
    CHECK(text_of(where) == "<probe>\n    <panel scale=\"0.0\" label=\"-0\"/>\n</probe>\n");
}

TEST_CASE("saving a negative zero a second time writes nothing more", "[config]")
{
    const std::filesystem::path where = scratch("twice.xml");
    const binding bound               = authored(where);
    const std::vector<edit> zero{edit{"panel/scale", "-0"}};
    REQUIRE(save(bound, zero).has_value());
    REQUIRE(text_of(where) == written_as_zero);

    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(bound, zero); });
    INFO(reported);
    REQUIRE(saved->has_value());
    CHECK(reported.find("0 value(s) written") != std::string::npos);
    CHECK(text_of(where) == written_as_zero);
}

TEST_CASE("two windows offering -0 then 0 gather as one edit carrying -0, which saves as 0", "[config]")
{
    const std::filesystem::path where = scratch("two-windows.xml");
    const binding bound               = authored(where);
    const offering first({edit{"panel/scale", "-0"}});
    const offering second({edit{"panel/scale", "0"}});
    const std::vector<edit> offered = shown_edits(std::array<const configurable *, 2>{&first, &second}, load_or_defaults(bound).values);
    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().kind == edit_kind::bound);
    CHECK(offered.front().value == "-0");

    REQUIRE(save(bound, offered).has_value());
    CHECK(text_of(where) == written_as_zero);
}
