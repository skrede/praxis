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
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <utility>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

constexpr std::string_view hand_written = "<probe>\n"
                                          "    <panel scale=\"1.5\" zero=\"0\" one=\"1\" label=\"x\"/>\n"
                                          "</probe>\n";

binding authored(const std::filesystem::path &where)
{
    std::filesystem::create_directories(where.parent_path());
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << hand_written;

    declaration shape("probe");
    shape.group("panel").field("panel/scale", field_kind::real, "1.0").field("panel/zero", field_kind::real, "0").field("panel/one", field_kind::real, "1");
    shape.field("panel/label", field_kind::text, "");
    return binding{std::move(shape), resolve(where, where.parent_path()), expectation::complete};
}

std::filesystem::path scratch(const std::string &name)
{
    return std::filesystem::temp_directory_path() / "praxis-config-non-finite" / name;
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

std::vector<edit> shown_by(const configurable &window, const document &carried)
{
    return shown_edits(std::array<const configurable *, 1>{&window}, carried);
}

bool unsaved(const configurable &window, const document &carried)
{
    return anything_unsaved(std::array<const configurable *, 1>{&window}, carried);
}

}

TEST_CASE("a NaN or infinite offer for a real key is left unwritten with a warning naming it, while every other edit saves and nothing is left unsaved", "[config]")
{
    const std::filesystem::path where = scratch("beside.xml");
    const binding bound               = authored(where);
    const document carried            = load_or_defaults(bound).values;

    const narrowing narrowed({edit{"panel/scale", "nan"}, edit{"panel/one", "2"}});
    const offering fixed({edit{"panel/zero", "inf"}});
    const std::array<const configurable *, 2> shown{&narrowed, &fixed};
    std::vector<edit> offered;
    const std::string said = praxis::tests::reported_by([&] { offered = shown_edits(shown, carried); });
    INFO(said);
    REQUIRE(offered.size() == 1u);
    CHECK((offered.front().key == "panel/one" && offered.front().kind == edit_kind::bound && offered.front().value == "2"));
    CHECK(std::ranges::none_of(offered, [](const edit &one) { return one.kind == edit_kind::refused; }));
    CHECK(said.find("'panel/scale' is offered as 'nan'") != std::string::npos);
    CHECK(said.find("'panel/zero' is offered as 'inf'") != std::string::npos);

    REQUIRE(save(bound, offered).has_value());
    const document reloaded = load_or_defaults(bound).values;
    CHECK(reloaded.real("panel/scale").value_or(0.0) == 1.5);
    CHECK(reloaded.real("panel/zero").value_or(-1.0) == 0.0);
    CHECK(reloaded.real("panel/one").value_or(0.0) == 2.0);
    CHECK(text_of(where).find("scale=\"1.5\"") != std::string::npos);
    CHECK(text_of(where).find("zero=\"0\"") != std::string::npos);

    CHECK_FALSE(unsaved(narrowed, reloaded));
    CHECK_FALSE(unsaved(narrowing({edit{"panel/scale", "-inf"}}), carried));
    CHECK_FALSE(unsaved(offering({edit{"panel/scale", "nan"}}), carried));
}

TEST_CASE("every spelling of NaN or an infinity is dropped, and a text that is not a number is offered as before", "[config]")
{
    const document carried = load_or_defaults(authored(scratch("spellings.xml"))).values;

    const std::array<std::string, 13> dropped{"nan", "-nan", "+nan", "NaN", "inf", "-inf", "+inf", "INF", "infinity", "-Infinity", "-nan(ind)", "nan(snan)", " nan "};
    for(const std::string &spelled : dropped)
    {
        INFO("'" << spelled << "'");
        std::vector<edit> offered;
        const std::string said = praxis::tests::reported_by([&] { offered = shown_by(offering({edit{"panel/scale", spelled}}), carried); });
        CHECK(offered.empty());
        CHECK(said.find("'panel/scale'") != std::string::npos);
    }

    const std::array<std::pair<std::string, std::string>, 5> kept{
            {{"panel/scale", "nanx"}, {"panel/scale", "infinite"}, {"panel/scale", "1e999"}, {"panel/scale", "abc"}, {"panel/label", "nan"}}};
    for(const auto &[key, spelled] : kept)
    {
        INFO("'" << key << "' offered as '" << spelled << "'");
        std::vector<edit> offered;
        const std::string said = praxis::tests::reported_by([&] { offered = shown_by(offering({edit{key, spelled}}), carried); });
        CHECK(said.empty());
        REQUIRE(offered.size() == 1u);
        CHECK((offered.front().key == key && offered.front().kind == edit_kind::bound && offered.front().value == spelled));
    }
}

TEST_CASE("a NaN or an infinity handed to a save directly is refused and the document is left as it was", "[config]")
{
    const std::array<std::string, 3> non_finite{"nan", "inf", "-inf"};
    for(const std::string &spelled : non_finite)
    {
        INFO("'" << spelled << "'");
        const std::filesystem::path where = scratch("direct.xml");
        const binding bound               = authored(where);

        const expected<void, error> written = save(bound, std::vector<edit>{edit{"panel/scale", spelled}, edit{"panel/one", "2"}});
        REQUIRE_FALSE(written.has_value());
        CHECK(written.error().code == error_code::rejected_content);
        CHECK(text_of(where) == hand_written);
        CHECK_FALSE(std::filesystem::exists(where.parent_path() / ("." + where.filename().string() + ".staging")));
    }
}
