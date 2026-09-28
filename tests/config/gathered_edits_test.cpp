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
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

constexpr std::string_view hand_written = "<probe>\n"
                                          "    <panel scale=\"1.5\"/>\n"
                                          "</probe>\n";

binding authored(const std::filesystem::path &where)
{
    std::filesystem::create_directories(where.parent_path());
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << hand_written;

    declaration shape("probe");
    shape.group("panel").field("panel/scale", field_kind::real, "1.0");
    return binding{std::move(shape), resolve(where, where.parent_path()), expectation::complete};
}

std::filesystem::path scratch(const std::string &name)
{
    return std::filesystem::temp_directory_path() / "praxis-config-gathered" / name;
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

private:
    std::vector<edit> m_offered;
};

std::vector<edit> gathered(const offering &first, const offering &second, const document &carried)
{
    const std::array<const configurable *, 2> shown{&first, &second};
    return shown_edits(shown, carried);
}

bool anything_unsaved_in(const offering &first, const offering &second, const document &carried)
{
    const std::array<const configurable *, 2> shown{&first, &second};
    return anything_unsaved(shown, carried);
}

}

TEST_CASE("an edit two shown implementors offer alike is gathered once and saved once", "[config]")
{
    const binding bound    = authored(scratch("alike.xml"));
    const document carried = load_or_defaults(bound).values;

    const offering first({edit{"panel/scale", "2.5"}});
    const offering second({edit{"panel/scale", "2.5"}});
    const std::vector<edit> offered = gathered(first, second, carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().kind == edit_kind::bound);

    REQUIRE(save(bound, offered).has_value());
    CHECK(load_or_defaults(bound).values.real("panel/scale").value_or(0.0) == 2.5);
}

TEST_CASE("two values offered for one key are gathered as one refused edit naming the key and nothing is written", "[config]")
{
    const std::filesystem::path where = scratch("differing.xml");
    const binding bound               = authored(where);
    const document carried            = load_or_defaults(bound).values;

    const offering first({edit{"panel/scale", "2.5"}});
    const offering second({edit{"panel/scale", "3.5"}});
    const std::vector<edit> offered = gathered(first, second, carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().kind == edit_kind::refused);
    CHECK(offered.front().key == "panel/scale");
    CHECK(offered.front().value.find("2.5") != std::string::npos);
    CHECK(offered.front().value.find("3.5") != std::string::npos);

    const expected<void, error> written = save(bound, offered);
    REQUIRE_FALSE(written.has_value());
    CHECK(written.error().code == error_code::rejected_content);
    CHECK(written.error().message.find("panel/scale") != std::string::npos);
    CHECK(text_of(where) == hand_written);
    CHECK(anything_unsaved_in(first, second, carried));
}

TEST_CASE("gathered edits keep the order they were first offered in", "[config]")
{
    const document carried = load_or_defaults(authored(scratch("ordered.xml"))).values;

    const offering first({edit{"panel/a", "1"}, edit{"panel/b", "2"}});
    const offering second({edit{"panel/b", "2"}, edit{"panel/c", "3"}});
    const std::vector<edit> offered = gathered(first, second, carried);

    std::vector<std::string> keys;
    for(const edit &one : offered)
        keys.push_back(one.key);
    CHECK(keys == std::vector<std::string>{"panel/a", "panel/b", "panel/c"});
}

TEST_CASE("a removal or a refusal offered twice alike is gathered once", "[config]")
{
    const document carried = load_or_defaults(authored(scratch("removed.xml"))).values;

    const std::vector<edit> both{edit{"panel/item", "first", edit_kind::taken_out}, edit{"panel/model", "no root holds it", edit_kind::refused}};
    const offering first(both);
    const offering second(both);
    const std::vector<edit> offered = gathered(first, second, carried);

    REQUIRE(offered.size() == 2u);
    CHECK(offered[0].kind == edit_kind::taken_out);
    CHECK(offered[0].key == "panel/item");
    CHECK(offered[1].kind == edit_kind::refused);
    CHECK(offered[1].key == "panel/model");
}
