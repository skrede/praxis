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
#include <cmath>
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
                                          "    <panel scale=\"1.5\" zero=\"0\" one=\"1\" shown=\"false\" count=\"7\" label=\"0.1\"/>\n"
                                          "</probe>\n";

binding authored(const std::filesystem::path &where)
{
    std::filesystem::create_directories(where.parent_path());
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << hand_written;

    declaration shape("probe");
    shape.group("panel").field("panel/scale", field_kind::real, "1.0").field("panel/zero", field_kind::real, "0").field("panel/one", field_kind::real, "1");
    shape.field("panel/shown", field_kind::flag, "false").field("panel/count", field_kind::integer, "0").field("panel/label", field_kind::text, "");
    return binding{std::move(shape), resolve(where, where.parent_path()), expectation::complete};
}

std::filesystem::path scratch(const std::string &name)
{
    return std::filesystem::temp_directory_path() / "praxis-config-spelled" / name;
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

std::vector<edit> gathered(const configurable &first, const configurable &second, const document &carried)
{
    const std::array<const configurable *, 2> shown{&first, &second};
    return shown_edits(shown, carried);
}

}

TEST_CASE("one value offered by two windows in two spellings is gathered as one edit carrying the text offered first, and saved once", "[config]")
{
    struct spelled_twice
    {
        std::string first;
        std::string second;
        double reads;
    };
    const std::array<spelled_twice, 6> pairs{{{"0.10", "0.1", 0.1}, {"0.1", "0.10", 0.1}, {"-0", "0", 0.0}, {"0", "-0", 0.0}, {"1", "1.0", 1.0}, {"1.0", "1", 1.0}}};
    for(const auto &[first, second, reads] : pairs)
    {
        INFO("'" << first << "' then '" << second << "'");
        const binding bound             = authored(scratch("one-value.xml"));
        const document carried          = load_or_defaults(bound).values;
        const std::vector<edit> offered = gathered(offering({edit{"panel/scale", first}}), offering({edit{"panel/scale", second}}), carried);
        REQUIRE(offered.size() == 1u);
        CHECK(offered.front().kind == edit_kind::bound);
        CHECK(offered.front().value == first);

        REQUIRE(save(bound, offered).has_value());
        CHECK(load_or_defaults(bound).values.real("panel/scale").value_or(-1.0) == reads);
    }
}

TEST_CASE("two different values for one key stay one refused edit, however close, and nothing is written", "[config]")
{
    const std::array<std::pair<std::string, std::string>, 2> pairs{{{"0.1", "0.2"}, {exact_text(0.1), exact_text(std::nextafter(0.1, 1.0))}}};
    for(const auto &[first, second] : pairs)
    {
        INFO("'" << first << "' against '" << second << "'");
        const std::filesystem::path where = scratch("two-values.xml");
        const binding bound               = authored(where);
        const document carried            = load_or_defaults(bound).values;

        const std::vector<edit> offered = gathered(offering({edit{"panel/scale", first}}), offering({edit{"panel/scale", second}}), carried);
        REQUIRE(offered.size() == 1u);
        CHECK(offered.front().kind == edit_kind::refused);
        CHECK(offered.front().key == "panel/scale");
        CHECK(offered.front().value.find("'" + first + "'") != std::string::npos);
        CHECK(offered.front().value.find("'" + second + "'") != std::string::npos);

        const expected<void, error> written = save(bound, offered);
        REQUIRE_FALSE(written.has_value());
        CHECK(written.error().code == error_code::rejected_content);
        CHECK(text_of(where) == hand_written);
    }
}

TEST_CASE("offers for a flag, an integer and a text key are gathered as their kind reads them", "[config]")
{
    const document carried = load_or_defaults(authored(scratch("kinds.xml"))).values;

    const offering first({edit{"panel/shown", "1"}, edit{"panel/count", "07"}, edit{"panel/label", "0.1"}});
    const offering second({edit{"panel/shown", "true"}, edit{"panel/count", "7"}, edit{"panel/label", "0.10"}});
    const std::vector<edit> offered = gathered(first, second, carried);

    REQUIRE(offered.size() == 3u);
    CHECK((offered[0].key == "panel/shown" && offered[0].kind == edit_kind::bound && offered[0].value == "1"));
    CHECK((offered[1].key == "panel/count" && offered[1].kind == edit_kind::bound && offered[1].value == "07"));
    CHECK((offered[2].key == "panel/label" && offered[2].kind == edit_kind::refused));
}

TEST_CASE("a document carrying 0 and 1 is not left unsaved by an offered -0 and 1.0", "[config]")
{
    const document carried = load_or_defaults(authored(scratch("unsaved.xml"))).values;

    const std::vector<edit> spelled{edit{"panel/zero", "-0"}, edit{"panel/one", "1.0"}, edit{"panel/scale", "1.50"}, edit{"panel/shown", "0"}, edit{"panel/count", "07"}};
    CHECK(unsaved_edits(carried, spelled).empty());
    const narrowing window(spelled);
    CHECK_FALSE(anything_unsaved(std::array<const configurable *, 1>{&window}, carried));

    const std::array<const configurable *, 1> absent{nullptr};
    CHECK(shown_edits(std::span<const configurable *const>(), carried).empty());
    CHECK_FALSE(anything_unsaved(std::span<const configurable *const>(), carried));
    CHECK(shown_edits(absent, carried).empty());
    CHECK_FALSE(anything_unsaved(absent, carried));

    const std::vector<edit> other{edit{"panel/scale", exact_text(std::nextafter(1.5, 2.0))}, edit{"panel/label", "0.10"}, edit{"panel/scale", "abc"}, edit{"panel/scale", ""}};
    CHECK(unsaved_edits(carried, other).size() == 4u);

    const std::vector<edit> unreadable = gathered(offering({edit{"panel/scale", "abc"}}), offering({edit{"panel/scale", "abc"}}), carried);
    REQUIRE(unreadable.size() == 1u);
    CHECK((unreadable.front().kind == edit_kind::bound && unreadable.front().value == "abc"));

    const std::filesystem::path unreadable_where = scratch("unreadable.xml");
    const expected<void, error> refused          = save(authored(unreadable_where), unreadable);
    REQUIRE_FALSE(refused.has_value());
    CHECK(refused.error().code == error_code::rejected_content);
    CHECK(refused.error().message.find("panel/scale") != std::string::npos);
    CHECK(refused.error().message.find("abc") != std::string::npos);
    CHECK(text_of(unreadable_where) == hand_written);
}

TEST_CASE("a save's read-back accepts a value spelled otherwise than it reads back and refuses one that reads back as nothing", "[config]")
{
    const std::filesystem::path spelled_where = scratch("spelled.xml");
    const binding spelled                     = authored(spelled_where);
    REQUIRE(save(spelled, std::vector<edit>{edit{"panel/scale", "1.50"}}).has_value());
    CHECK(text_of(spelled_where).find("scale=\"1.50\"") != std::string::npos);
    CHECK(load_or_defaults(spelled).values.real("panel/scale").value_or(0.0) == 1.5);

    const std::filesystem::path broken_where = scratch("broken.xml");
    const binding broken                     = authored(broken_where);
    const expected<void, error> written      = save(broken, std::vector<edit>{edit{"panel/scale", "1.5x"}});
    REQUIRE_FALSE(written.has_value());
    CHECK(written.error().code == error_code::rejected_content);
    CHECK(text_of(broken_where) == hand_written);
}
