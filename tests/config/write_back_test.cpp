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
#include <array>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <utility>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

// A leading comment, a comment between two elements, a blank line, two siblings in an order neither
// the declaration nor the alphabet implies, one leaf carried as a text child among attributes, and
// four-space indentation: everything a rendered document loses.
constexpr std::string_view hand_written = "<!-- The main window as the operator left it. Sizes are in pixels. -->\n"
                                          "<probe>\n"
                                          "    <!-- Height first, because the display it was tuned on is portrait. -->\n"
                                          "    <window height=\"720\" width=\"1280\" mode=\"docked\">\n"
                                          "        <title>Overview</title>\n"
                                          "    </window>\n"
                                          "\n"
                                          "    <stations>\n"
                                          "        <station name=\"alpha\"><panel scale=\"1.5\"/></station>\n"
                                          "        <station name=\"beta\"><panel scale=\"2.5\"/></station>\n"
                                          "    </stations>\n"
                                          "</probe>\n";

// The same document with the enumerated leaf left out, so that leaf reads its declared fallback and
// the file is not where its value came from.
constexpr std::string_view without_a_mode = "<probe>\n"
                                            "    <window height=\"720\" width=\"1280\">\n"
                                            "        <title>Overview</title>\n"
                                            "    </window>\n"
                                            "</probe>\n";

// One element carrying neither of the two values a save writes into it, so both are placed at the
// same byte and the order they come out in is the splicer's own.
constexpr std::string_view a_bare_window = "<probe>\n"
                                           "    <window height=\"720\"/>\n"
                                           "</probe>\n";

// A document carrying no window element at all, so the element two values hang under has to be made
// before either of them has anywhere to go.
constexpr std::string_view without_a_window = "<probe>\n"
                                              "    <stations>\n"
                                              "        <station name=\"alpha\"><panel scale=\"1.5\"/></station>\n"
                                              "    </stations>\n"
                                              "</probe>\n";

// A document standing open at nothing, which is as little as a document handed out one preset at a
// time can carry and still name what it is.
constexpr std::string_view an_empty_document = "<probe/>\n";

// A root standing open across two lines with nothing but blanks between them, which is what a
// hand-formatted document is left as once everything under it has been taken out.
constexpr std::string_view a_blank_root = "<probe>\n"
                                          "</probe>\n";

// A document closing two elements on one line, so the whitespace a new last child is placed against
// is not the whole of the line the parent's end tag sits on.
constexpr std::string_view a_compact_close = "<probe>\n"
                                             "    <stations>\n"
                                             "        <station name=\"alpha\"><panel scale=\"1.5\"/></station>\n"
                                             "    </stations></probe>\n";

// The leaf standing open at nothing, which is a document saying the value is empty rather than a
// document saying nothing about that value at all.
constexpr std::string_view an_empty_title = "<probe>\n"
                                            "    <window height=\"720\"><title/></window>\n"
                                            "</probe>\n";

// Two instances carrying one identity, standing apart with another between them.
constexpr std::string_view a_name_carried_twice = "<probe>\n"
                                                  "    <stations>\n"
                                                  "        <station name=\"alpha\"><panel scale=\"1.5\"/></station>\n"
                                                  "        <station name=\"beta\"><panel scale=\"2.5\"/></station>\n"
                                                  "        <station name=\"alpha\"><panel scale=\"3.5\"/></station>\n"
                                                  "    </stations>\n"
                                                  "</probe>\n";

// A collection whose instances each hold a collection of their own.
constexpr std::string_view slots_in_stations = "<probe>\n"
                                               "    <stations>\n"
                                               "        <station name=\"alpha\"><slot id=\"x\"/></station>\n"
                                               "        <station name=\"beta\"><slot id=\"y\"/></station>\n"
                                               "    </stations>\n"
                                               "</probe>\n";

// Two instances each holding a collection of their own, the first holding four instances on lines of
// one length.
constexpr std::string_view slots_under_two_stations = "<probe>\n"
                                                      "    <stations>\n"
                                                      "        <station name=\"alpha\">\n"
                                                      "            <slot id=\"a\" v=\"1\"/>\n"
                                                      "            <slot id=\"b\" v=\"2\"/>\n"
                                                      "            <slot id=\"c\" v=\"3\"/>\n"
                                                      "            <slot id=\"d\" v=\"4\"/>\n"
                                                      "        </station>\n"
                                                      "        <station name=\"beta\"><slot id=\"e\" v=\"5\"/></station>\n"
                                                      "    </stations>\n"
                                                      "</probe>\n";

// Two instances whose own collections both carry one identity.
constexpr std::string_view one_identity_under_two_stations = "<probe>\n"
                                                             "    <stations>\n"
                                                             "        <station name=\"alpha\">\n"
                                                             "            <slot id=\"x\" v=\"1\"/>\n"
                                                             "            <slot id=\"y\" v=\"2\"/>\n"
                                                             "        </station>\n"
                                                             "        <station name=\"beta\">\n"
                                                             "            <slot id=\"x\" v=\"3\"/>\n"
                                                             "        </station>\n"
                                                             "    </stations>\n"
                                                             "</probe>\n";

// Two instances standing on one line.
constexpr std::string_view stations_on_one_line = "<probe>\n"
                                                  "    <stations>\n"
                                                  "        <station name=\"alpha\"><panel scale=\"1.5\"/></station><station name=\"beta\"><panel scale=\"2.5\"/></station>\n"
                                                  "    </stations>\n"
                                                  "</probe>\n";

// An instance alone on its line followed by two blanks.
constexpr std::string_view a_trailing_blank = "<probe>\n"
                                              "    <stations>\n"
                                              "        <station name=\"alpha\"><panel scale=\"1.5\"/></station>  \n"
                                              "        <station name=\"beta\"><panel scale=\"2.5\"/></station>\n"
                                              "    </stations>\n"
                                              "</probe>\n";

// The groups are declared in an order the document does not follow, so the document's own ordering
// is nobody's default and survives only by not being touched.
declaration described()
{
    declaration shape("probe");
    shape.group("stations")
            .collection("stations/station", "name")
            .group("stations/station/panel")
            .field("stations/station/panel/scale", field_kind::real, "1.0")
            .group("window")
            .field("window/title", field_kind::text, "untitled")
            .field("window/width", field_kind::integer, "800")
            .field("window/height", field_kind::integer, "600")
            .choice("window/mode", {"docked", "floating"}, "docked");
    return shape;
}

declaration nested()
{
    declaration shape("probe");
    shape.group("stations").collection("stations/station", "name").collection("stations/station/slot", "id").field("stations/station/slot/v", field_kind::integer, "0");
    return shape;
}

// An implementor standing for exactly the edits it was handed, whatever the document carries.
class offering final : public configurable
{
public:
    explicit offering(std::vector<edit> held)
            : m_held(std::move(held))
    {
    }

    std::string_view settings_path() const override
    {
        return "stations";
    }

    std::vector<edit> settings_edits(const document &) const override
    {
        return m_held;
    }

private:
    std::vector<edit> m_held;
};

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-write-back";
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

// A path in a corner of its own with nothing at it, so what a save leaves beside a document can be
// counted rather than looked for under a name only the writer knows.
std::filesystem::path nothing_at(const std::string &corner)
{
    const std::filesystem::path directory = scratch() / corner;
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    return directory / "settings.xml";
}

std::filesystem::path alone_in(const std::string &corner, std::string_view text)
{
    const std::filesystem::path where = nothing_at(corner);
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << text;
    return where;
}

std::size_t entries_in(const std::filesystem::path &directory)
{
    std::size_t counted = 0;
    for(const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(directory))
    {
        static_cast<void>(entry);
        ++counted;
    }
    return counted;
}

std::size_t occurrences(const std::string &text, const std::string &needle)
{
    std::size_t counted = 0;
    for(std::size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + needle.size()))
        ++counted;
    return counted;
}

std::string text_of(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    std::ostringstream all;
    all << in.rdbuf();
    return all.str();
}

std::string with_carriage_returns(std::string_view text)
{
    std::string ended;
    for(const char letter : text)
    {
        if(letter == '\n')
            ended.push_back('\r');
        ended.push_back(letter);
    }
    return ended;
}

// `text` with the first `before` it carries replaced by `after`.
std::string with_replaced(std::string text, std::string_view before, std::string_view after)
{
    const std::size_t at = text.find(before);
    REQUIRE(at != std::string::npos);
    return text.replace(at, before.size(), after);
}

// `slots_under_two_stations` without slot `a`, and slot `c` carrying 9.
std::string without_a_and_c_at_nine()
{
    const std::string without_a = with_replaced(std::string(slots_under_two_stations), "\n            <slot id=\"a\" v=\"1\"/>", "");
    return with_replaced(without_a, "<slot id=\"c\" v=\"3\"/>", "<slot id=\"c\" v=\"9\"/>");
}

std::string why(const expected<void, error> &outcome)
{
    return outcome.has_value() ? std::string() : outcome.error().message;
}

// The one stretch two texts disagree over, taken from both ends, so what is left between the common
// prefix and the common suffix is by construction the only region that differs.
struct difference
{
    std::size_t at;
    std::string before;
    std::string after;
};

difference between(const std::string &before, const std::string &after)
{
    std::size_t head = 0;
    while(head < before.size() && head < after.size() && before[head] == after[head])
        ++head;

    std::size_t tail = 0;
    while(tail < before.size() - head && tail < after.size() - head && before[before.size() - 1 - tail] == after[after.size() - 1 - tail])
        ++tail;

    return difference{head, before.substr(head, before.size() - head - tail), after.substr(head, after.size() - head - tail)};
}

// The digits of a printed real that carry its value, which is every digit after the leading zeros
// and before any exponent.
std::size_t significant(std::string_view printed)
{
    std::size_t counted = 0;
    bool leading        = true;
    for(const char letter : printed.substr(0, printed.find('e')))
    {
        if(letter < '0' || letter > '9')
            continue;
        leading = leading && letter == '0';
        if(!leading)
            ++counted;
    }
    return counted;
}

}

TEST_CASE("a save edits one value and leaves every byte the author wrote around it", "[config]")
{
    const std::filesystem::path where = authored("preserved.xml", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> one{edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string after      = text_of(where);
    const difference disagreeing = between(before, after);
    REQUIRE(disagreeing.before == "1280");
    REQUIRE(disagreeing.after == "2464");

    std::string only_that_value = before;
    only_that_value.replace(disagreeing.at, disagreeing.before.size(), disagreeing.after);
    REQUIRE(after == only_that_value);

    REQUIRE(after.find("<!-- The main window as the operator left it.") != std::string::npos);
    REQUIRE(after.find("<!-- Height first,") != std::string::npos);
    REQUIRE(after.find("</window>\n\n    <stations>") != std::string::npos);
    REQUIRE(after.find("<window") < after.find("<stations>"));
}

TEST_CASE("a save of values the document already carries leaves it byte for byte as it was", "[config]")
{
    const std::filesystem::path where = authored("idempotent.xml", hand_written);
    const location at                 = resolve(where, scratch());

    const std::vector<edit> changes{edit{"window/width", "2464"}, edit{"window/title", "Console"}};
    const expected<void, error> first = save(described(), at, changes);
    INFO(why(first));
    REQUIRE(first.has_value());

    const std::string once = text_of(where);
    REQUIRE(once.find("<title>Console</title>") != std::string::npos);

    const expected<void, error> again = save(described(), at, changes);
    INFO(why(again));
    REQUIRE(again.has_value());
    REQUIRE(text_of(where) == once);

    const std::vector<edit> as_it_stands{edit{"window/height", "720"}, edit{"window/mode", "docked"}};
    const expected<void, error> untouched = save(described(), at, as_it_stands);
    INFO(why(untouched));
    REQUIRE(untouched.has_value());
    REQUIRE(text_of(where) == once);
}

TEST_CASE("a real value written with the digits it needs reads back bit for bit", "[config]")
{
    const std::filesystem::path where = authored("precision.xml", hand_written);
    const location at                 = resolve(where, scratch());

    const double not_representable = 0.1 + 0.2;
    const std::string written      = exact_text(not_representable);
    REQUIRE(written == "0.30000000000000004");
    REQUIRE(significant(written) == std::numeric_limits<double>::max_digits10);

    const std::vector<edit> one{edit{"stations/station[0]/panel/scale", written}};
    const expected<void, error> saved = save(described(), at, one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const expected<document, error> reloaded = load(described(), at);
    REQUIRE(reloaded.has_value());

    const expected<double, error> read = reloaded.value().real("stations/station[0]/panel/scale");
    REQUIRE(read.has_value());
    REQUIRE(read.value() == not_representable);
}

TEST_CASE("a value edited under one instance leaves the value under the other exactly as it was", "[config]")
{
    const std::filesystem::path where = authored("instances.xml", hand_written);
    const location at                 = resolve(where, scratch());
    const std::string before          = text_of(where);

    const expected<document, error> held = load(described(), at);
    REQUIRE(held.has_value());
    const expected<std::string, error> keyed = held.value().key("stations/station", "beta", "panel/scale");
    REQUIRE(keyed.has_value());

    const std::vector<edit> one{edit{keyed.value(), "7.25"}};
    const expected<void, error> saved = save(described(), at, one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string as_authored = "scale=\"2.5\"";
    std::string only_that_value   = before;
    only_that_value.replace(before.find(as_authored), as_authored.size(), "scale=\"7.25\"");
    REQUIRE(text_of(where) == only_that_value);
    REQUIRE(text_of(where).find("<station name=\"alpha\"><panel scale=\"1.5\"/></station>") != std::string::npos);

    const expected<document, error> reloaded = load(described(), at);
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().real(reloaded.value().key("stations/station", "beta", "panel/scale").value()).value() == 7.25);
    REQUIRE(reloaded.value().real(reloaded.value().key("stations/station", "alpha", "panel/scale").value()).value() == 1.5);
}

TEST_CASE("a key naming an instance the document does not carry is refused by name and nothing at all is written", "[config]")
{
    const std::filesystem::path where = authored("unlocatable.xml", hand_written);
    const location at                 = resolve(where, scratch());
    const std::string before          = text_of(where);

    const std::vector<edit> beyond{edit{"window/width", "2464"}, edit{"stations/station[2]/panel/scale", "9.5"}};
    const expected<void, error> saved = save(described(), at, beyond);
    REQUIRE_FALSE(saved.has_value());
    REQUIRE(saved.error().code == error_code::unlocatable_key);
    REQUIRE(saved.error().message.find("stations/station[2]/panel/scale") != std::string::npos);
    REQUIRE(text_of(where) == before);
}

TEST_CASE("an instance the same save names the identity of is created and written into", "[config]")
{
    const std::filesystem::path where = authored("created-instance.xml", without_a_window);
    const location at                 = resolve(where, scratch());

    const std::vector<edit> naming{edit{"stations/station[1]/name", "gamma"}, edit{"stations/station[1]/panel/scale", "9.5"}, edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), at, naming);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string after = text_of(where);
    INFO(after);
    REQUIRE(occurrences(after, "<station ") == 2);

    const expected<document, error> reloaded = load(described(), at);
    REQUIRE(reloaded.has_value());

    const std::vector<std::string> named = reloaded.value().identities("stations/station");
    REQUIRE(named.size() == 2);
    REQUIRE(named[0] == "alpha");
    REQUIRE(named[1] == "gamma");

    const expected<std::string, error> keyed = reloaded.value().key("stations/station", "gamma", "panel/scale");
    REQUIRE(keyed.has_value());
    REQUIRE(reloaded.value().real(keyed.value()).value() == 9.5);
}

TEST_CASE("an instance the same save names by the empty string is not created and nothing at all is written", "[config]")
{
    const std::filesystem::path where = authored("empty-identity.xml", without_a_window);
    const location at                 = resolve(where, scratch());
    const std::string before          = text_of(where);

    const std::vector<edit> unnamed{edit{"stations/station[1]/name", ""}, edit{"stations/station[1]/panel/scale", "9.5"}, edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), at, unnamed);
    INFO(why(saved));
    REQUIRE_FALSE(saved.has_value());
    REQUIRE(saved.error().code == error_code::unlocatable_key);
    REQUIRE(saved.error().message.find("stations/station[1]/panel/scale") != std::string::npos);

    const std::string after = text_of(where);
    INFO(after);
    REQUIRE(after == before);
    REQUIRE(occurrences(after, "<station ") == 1);

    const expected<document, error> reloaded = load(described(), at);
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().identities("stations/station").size() == 1);
}

TEST_CASE("a value an element the document carries writes nowhere is inserted into that element's start tag", "[config]")
{
    const std::filesystem::path where = authored("inserted.xml", without_a_mode);
    const location at                 = resolve(where, scratch());
    const std::string before          = text_of(where);

    const std::vector<edit> one{edit{"window/mode", "floating"}};
    const expected<void, error> saved = save(described(), at, one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string after      = text_of(where);
    const difference disagreeing = between(before, after);
    REQUIRE(disagreeing.before.empty());
    REQUIRE(disagreeing.after == " mode=\"floating\"");

    std::string only_that_insertion = before;
    only_that_insertion.insert(disagreeing.at, disagreeing.after);
    REQUIRE(after == only_that_insertion);

    const expected<document, error> reloaded = load(described(), at);
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().text("window/mode").value() == "floating");
}

TEST_CASE("two values written into one start tag come out in the same order whatever standard library spliced them", "[config]")
{
    const std::filesystem::path where = authored("ordering.xml", a_bare_window);

    const std::vector<edit> two{edit{"window/mode", "floating"}, edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), two);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    REQUIRE(text_of(where) ==
            "<probe>\n"
            "    <window height=\"720\" width=\"2464\" mode=\"floating\"/>\n"
            "</probe>\n");
}

TEST_CASE("two values hanging under an element the document does not carry create that element once", "[config]")
{
    const std::filesystem::path where = authored("created.xml", without_a_window);

    const std::vector<edit> two{edit{"window/mode", "floating"}, edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), two);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string after = text_of(where);
    REQUIRE(after ==
            "<probe>\n"
            "    <stations>\n"
            "        <station name=\"alpha\"><panel scale=\"1.5\"/></station>\n"
            "    </stations>\n"
            "    <window width=\"2464\" mode=\"floating\"/>\n"
            "</probe>\n");
    REQUIRE(occurrences(after, "<window") == 1);
}

TEST_CASE("a document standing open at nothing is opened around the element a save has to make in it", "[config]")
{
    const std::filesystem::path where = authored("empty.xml", an_empty_document);

    const std::vector<edit> one{edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    REQUIRE(text_of(where) ==
            "<probe>\n"
            "<window width=\"2464\"/>\n"
            "</probe>\n");
}

TEST_CASE("a value hanging under a parent carrying nothing but blanks is written into it and the document still reads", "[config]")
{
    const std::filesystem::path where = authored("blank-parent.xml", a_blank_root);

    const std::vector<edit> one{edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    REQUIRE(text_of(where) ==
            "<probe>\n"
            "<window width=\"2464\"/>\n"
            "</probe>\n");
}

TEST_CASE("an element created under a parent whose end tag shares a line with a sibling lands at that sibling's column", "[config]")
{
    const std::filesystem::path where = authored("compact-close.xml", a_compact_close);

    const std::vector<edit> one{edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    REQUIRE(text_of(where) ==
            "<probe>\n"
            "    <stations>\n"
            "        <station name=\"alpha\"><panel scale=\"1.5\"/></station>\n"
            "    </stations>\n"
            "    <window width=\"2464\"/></probe>\n");
}

TEST_CASE("an empty value saved into a key the document has no place for is written rather than taken for what it already says", "[config]")
{
    const std::filesystem::path where = authored("empty-into-absent.xml", a_bare_window);
    const location at                 = resolve(where, scratch());

    const std::vector<edit> one{edit{"window/title", ""}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(described(), at, one); });
    INFO(reported);
    REQUIRE(saved->has_value());
    REQUIRE(reported.find("1 value(s) written") != std::string::npos);

    const expected<document, error> reloaded = load(described(), at);
    REQUIRE(reloaded.has_value());
    const expected<std::string, error> read = reloaded.value().text("window/title");
    REQUIRE(read.has_value());
    REQUIRE(read.value().empty());
}

TEST_CASE("an empty value saved into a leaf the document carries standing open at nothing changes no byte of it", "[config]")
{
    const std::filesystem::path where = authored("empty-into-vacant.xml", an_empty_title);
    const location at                 = resolve(where, scratch());
    const std::string before          = text_of(where);

    const std::vector<edit> one{edit{"window/title", ""}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(described(), at, one); });
    INFO(reported);
    REQUIRE(saved->has_value());
    REQUIRE(reported.find("0 value(s) written") != std::string::npos);
    REQUIRE(text_of(where) == before);
}

TEST_CASE("a save whose read-back fails leaves the document as it was and nothing beside it", "[config]")
{
    const std::filesystem::path where = alone_in("read-back", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> outside_the_enumeration{edit{"window/mode", "hidden"}};
    const expected<void, error> saved = save(described(), resolve(where, where.parent_path()), outside_the_enumeration);
    REQUIRE_FALSE(saved.has_value());
    REQUIRE(saved.error().code == error_code::rejected_content);
    REQUIRE(saved.error().message.find("window/mode") != std::string::npos);
    REQUIRE(saved.error().message.find("hidden") != std::string::npos);
    REQUIRE(text_of(where) == before);
    REQUIRE(entries_in(where.parent_path()) == 1);
}

TEST_CASE("a save into a directory that cannot be written refuses and leaves nothing behind", "[config]")
{
    const std::filesystem::path where = alone_in("read-only", hand_written);
    const std::string before          = text_of(where);
    std::filesystem::permissions(where.parent_path(), std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec, std::filesystem::perm_options::replace);

    std::ofstream probing(where.parent_path() / "probe", std::ios::trunc);
    const bool refused = !probing.is_open();
    probing.close();

    const std::vector<edit> one{edit{"window/width", "2464"}};
    std::optional<expected<void, error>> saved;
    if(refused)
        saved = save(described(), resolve(where, where.parent_path()), one);
    std::filesystem::permissions(where.parent_path(), std::filesystem::perms::owner_all, std::filesystem::perm_options::replace);

    if(!saved)
    {
        SUCCEED("this process can write into a directory with no write permission, so there is nothing to refuse");
        return;
    }
    REQUIRE_FALSE(saved->has_value());
    REQUIRE(saved->error().code == error_code::unwritable_target);
    REQUIRE(text_of(where) == before);
    REQUIRE(entries_in(where.parent_path()) == 1);
}

TEST_CASE("one message at the informational level names the document a save landed in", "[config]")
{
    const std::filesystem::path where = alone_in("announced", hand_written);
    const location at                 = resolve(where, where.parent_path());

    const std::vector<edit> one{edit{"window/width", "2464"}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(described(), at, one); });
    INFO(reported);
    REQUIRE(saved->has_value());

    REQUIRE(occurrences(reported, at.resolved.string()) == 1);
    REQUIRE(reported.find("[info]") != std::string::npos);
    REQUIRE(reported.find("1 value(s) written") != std::string::npos);
    REQUIRE(reported.find(".staging") == std::string::npos);
    REQUIRE(entries_in(where.parent_path()) == 1);
}

TEST_CASE("a key the declaration does not name is refused by name and nothing at all is written", "[config]")
{
    const std::filesystem::path where = alone_in("undeclared", hand_written);
    const location at                 = resolve(where, where.parent_path());
    const std::string before          = text_of(where);

    const std::vector<edit> stray{edit{"window/width", "2464"}, edit{"gadget/size", "5"}};
    const expected<void, error> saved = save(described(), at, stray);
    REQUIRE_FALSE(saved.has_value());
    REQUIRE(saved.error().message.find("gadget/size") != std::string::npos);
    REQUIRE(text_of(where) == before);
    REQUIRE(entries_in(where.parent_path()) == 1);
}

TEST_CASE("a save where there is no document writes one from the declaration and the values into it", "[config]")
{
    const std::filesystem::path where = nothing_at("created-document");
    const location at                 = resolve(where, where.parent_path());

    const std::vector<edit> one{edit{"window/width", "2464"}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(described(), at, one); });
    INFO(reported);
    REQUIRE(saved->has_value());

    REQUIRE(std::filesystem::exists(where));
    REQUIRE(entries_in(where.parent_path()) == 1);
    REQUIRE(occurrences(reported, at.resolved.string()) == 1);

    const expected<document, error> reloaded = load(described(), at);
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().integer("window/width").value() == 2464);
    REQUIRE(reloaded.value().integer("window/height").value() == 600);
    REQUIRE(reloaded.value().origin_of("window/height").kind == origin_kind::source);
}

TEST_CASE("a save where there is no document and every value is the declared one still leaves the document", "[config]")
{
    const std::filesystem::path where = nothing_at("created-at-the-fallbacks");
    const location at                 = resolve(where, where.parent_path());

    const std::vector<edit> as_declared{edit{"window/width", "800"}, edit{"window/mode", "docked"}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(described(), at, as_declared); });
    INFO(reported);
    REQUIRE(saved->has_value());

    REQUIRE(std::filesystem::exists(where));
    REQUIRE(entries_in(where.parent_path()) == 1);
    REQUIRE(occurrences(reported, at.resolved.string()) == 1);
    REQUIRE(reported.find("0 value(s) written") != std::string::npos);
}

TEST_CASE("a save where there is no document and nothing can be written leaves nothing behind", "[config]")
{
    const std::filesystem::path where = nothing_at("created-read-only");
    std::filesystem::permissions(where.parent_path(), std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec, std::filesystem::perm_options::replace);

    std::ofstream probing(where.parent_path() / "probe", std::ios::trunc);
    const bool refused = !probing.is_open();
    probing.close();

    const std::vector<edit> one{edit{"window/width", "2464"}};
    std::optional<expected<void, error>> saved;
    if(refused)
        saved = save(described(), resolve(where, where.parent_path()), one);
    std::filesystem::permissions(where.parent_path(), std::filesystem::perms::owner_all, std::filesystem::perm_options::replace);

    if(!saved)
    {
        SUCCEED("this process can write into a directory with no write permission, so there is nothing to refuse");
        return;
    }
    REQUIRE_FALSE(saved->has_value());
    REQUIRE(saved->error().code == error_code::unwritable_target);
    REQUIRE_FALSE(std::filesystem::exists(where));
    REQUIRE(entries_in(where.parent_path()) == 0);
}

TEST_CASE("a save that changes nothing leaves the document alone and still says so once", "[config]")
{
    const std::filesystem::path where = alone_in("unchanged", hand_written);
    const location at                 = resolve(where, where.parent_path());
    const std::string before          = text_of(where);

    const std::vector<edit> as_it_stands{edit{"window/height", "720"}, edit{"window/mode", "docked"}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(described(), at, as_it_stands); });
    INFO(reported);
    REQUIRE(saved->has_value());

    REQUIRE(text_of(where) == before);
    REQUIRE(entries_in(where.parent_path()) == 1);
    REQUIRE(occurrences(reported, "[info]") == 1);
    REQUIRE(occurrences(reported, at.resolved.string()) == 1);
    REQUIRE(reported.find("0 value(s) written") != std::string::npos);
}

TEST_CASE("a save names the document once whether it replaced a value, inserted one or was held to the file", "[config]")
{
    const std::filesystem::path grown = alone_in("announced-insertion", without_a_mode);
    const location inserting          = resolve(grown, grown.parent_path());

    const std::vector<edit> one{edit{"window/mode", "floating"}};
    std::optional<expected<void, error>> inserted;
    const std::string about_insertion = praxis::tests::reported_by([&] { inserted = save(described(), inserting, one); });
    INFO(about_insertion);
    REQUIRE(inserted->has_value());
    REQUIRE(occurrences(about_insertion, inserting.resolved.string()) == 1);

    const std::filesystem::path held = alone_in("announced-policy", hand_written);
    const location holding           = resolve(held, held.parent_path());

    const std::vector<edit> carried{edit{"window/width", "2464"}};
    std::optional<expected<void, error>> file_backed;
    const std::string about_policy = praxis::tests::reported_by([&] { file_backed = save(described(), holding, carried, write_policy::file_backed_only); });
    INFO(about_policy);
    REQUIRE(file_backed->has_value());
    REQUIRE(occurrences(about_policy, holding.resolved.string()) == 1);
}

TEST_CASE("the file-backed policy is the only thing that leaves a value the document does not carry alone", "[config]")
{
    const std::filesystem::path where = alone_in("policy", without_a_mode);
    const location at                 = resolve(where, where.parent_path());
    const std::string before          = text_of(where);

    const std::vector<edit> one{edit{"window/mode", "floating"}};
    const expected<void, error> file_backed = save(described(), at, one, write_policy::file_backed_only);
    INFO(why(file_backed));
    REQUIRE(file_backed.has_value());
    REQUIRE(text_of(where) == before);

    const std::vector<edit> both{edit{"window/mode", "floating"}, edit{"window/width", "2464"}};
    const expected<void, error> beside = save(described(), at, both, write_policy::file_backed_only);
    INFO(why(beside));
    REQUIRE(beside.has_value());
    REQUIRE(text_of(where).find("width=\"2464\"") != std::string::npos);
    REQUIRE(text_of(where).find("mode=") == std::string::npos);

    const expected<void, error> every = save(described(), at, one, write_policy::every_edit);
    INFO(why(every));
    REQUIRE(every.has_value());
    REQUIRE(text_of(where).find("mode=\"floating\"") != std::string::npos);
}

TEST_CASE("an instance a save takes out goes whole and leaves every byte around it as it was", "[config]")
{
    const std::filesystem::path where = authored("taken-out.xml", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> one{edit{"stations/station", "alpha", edit_kind::taken_out}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string gone = "\n        <station name=\"alpha\"><panel scale=\"1.5\"/></station>";
    REQUIRE(before.find(gone) != std::string::npos);

    std::string only_that_station = before;
    only_that_station.erase(before.find(gone), gone.size());
    REQUIRE(text_of(where) == only_that_station);

    const expected<document, error> reloaded = load(described(), resolve(where, scratch()));
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().identities("stations/station") == std::vector<std::string>{"beta"});
}

// The keys a save carries address the document as it stands, so the instance the same save writes
// into is the one it named and not whichever instance took that ordinal once the other was gone.
TEST_CASE("a value written into an instance standing after one the same save takes out lands in the one it named", "[config]")
{
    const std::filesystem::path where = authored("taken-out-beside.xml", hand_written);

    const std::vector<edit> changes{edit{"stations/station", "alpha", edit_kind::taken_out}, edit{"stations/station[1]/panel/scale", "3.25"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const expected<document, error> reloaded = load(described(), resolve(where, scratch()));
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().identities("stations/station") == std::vector<std::string>{"beta"});
    REQUIRE(reloaded.value().real("stations/station[0]/panel/scale").value() == 3.25);
}

// An instance is appended at the ordinal the collection ends at, counted before anything is taken
// out of it, so the two have to agree about where the end is.
TEST_CASE("an instance created by the same save that takes another out is created once, at the end", "[config]")
{
    const std::filesystem::path where = authored("taken-out-and-made.xml", hand_written);

    const std::vector<edit> changes{edit{"stations/station", "alpha", edit_kind::taken_out}, edit{"stations/station[2]/name", "gamma"}, edit{"stations/station[2]/panel/scale", "4.5"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string after = text_of(where);
    REQUIRE(occurrences(after, "<station ") == 2u);

    const expected<document, error> reloaded = load(described(), resolve(where, scratch()));
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().identities("stations/station") == std::vector<std::string>{"beta", "gamma"});
    REQUIRE(reloaded.value().real("stations/station[1]/panel/scale").value() == 4.5);
}

TEST_CASE("a save naming an instance the document does not carry takes nothing out and leaves it as it was", "[config]")
{
    const std::filesystem::path where = authored("taken-out-absent.xml", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> one{edit{"stations/station", "gamma", edit_kind::taken_out}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());
    REQUIRE(text_of(where) == before);
}

TEST_CASE("a save asked to take an instance out of something the declaration names no collection at is refused by name", "[config]")
{
    const std::filesystem::path where = authored("taken-out-undeclared.xml", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> changes{edit{"window", "alpha", edit_kind::taken_out}, edit{"window/width", "2464"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), changes);
    REQUIRE_FALSE(saved.has_value());

    CHECK(saved.error().code == error_code::unlocatable_key);
    CHECK(saved.error().message.find("window") != std::string::npos);
    CHECK(text_of(where) == before);
}

// A value addressed to the instance a save takes out has nowhere left to land, and the instance
// taking that ordinal once the other is gone is not the one it named.
TEST_CASE("a save asked to write into an instance it also takes out is refused by name and nothing at all is written", "[config]")
{
    const std::filesystem::path where = authored("taken-out-and-written.xml", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> changes{edit{"stations/station", "alpha", edit_kind::taken_out}, edit{"stations/station[0]/panel/scale", "9.5"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), changes);
    REQUIRE_FALSE(saved.has_value());

    CHECK(saved.error().message.find("stations/station[0]/panel/scale") != std::string::npos);
    CHECK(text_of(where) == before);
}

// A segment with no bracket names the collection's first instance, for a removal as for any key.
TEST_CASE("a save writing into the first instance with no bracket while taking that instance out is refused by name", "[config]")
{
    const std::filesystem::path where = authored("taken-out-and-written-unbracketed.xml", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> changes{edit{"stations/station", "alpha", edit_kind::taken_out}, edit{"stations/station/panel/scale", "9.5"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), changes);
    REQUIRE_FALSE(saved.has_value());

    CHECK(saved.error().message.find("stations/station/panel/scale") != std::string::npos);
    CHECK(text_of(where) == before);
}

TEST_CASE("a removal naming an identity two instances carry takes both and every byte around them stays", "[config]")
{
    const std::filesystem::path where = authored("taken-out-twice.xml", a_name_carried_twice);
    const std::string before          = text_of(where);

    const std::vector<edit> changes{edit{"stations/station", "alpha", edit_kind::taken_out}, edit{"stations/station[1]/panel/scale", "4.5"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const expected<document, error> reloaded = load(described(), resolve(where, scratch()));
    REQUIRE(reloaded.has_value());
    CHECK(reloaded.value().identities("stations/station") == std::vector<std::string>{"beta"});
    CHECK(reloaded.value().real("stations/station[0]/panel/scale").value() == 4.5);
    CHECK(occurrences(text_of(where), "\n") == occurrences(before, "\n") - 2u);
}

TEST_CASE("a save that only takes an instance out says so in its one message", "[config]")
{
    const std::filesystem::path where = alone_in("announced-removal", hand_written);
    const location at                 = resolve(where, where.parent_path());

    const std::vector<edit> one{edit{"stations/station", "alpha", edit_kind::taken_out}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(described(), at, one); });
    INFO(reported);
    REQUIRE(saved->has_value());

    REQUIRE(occurrences(reported, "[info]") == 1);
    REQUIRE(occurrences(reported, at.resolved.string()) == 1);
    CHECK(reported.find("1 instance(s) taken out") != std::string::npos);
}

// Renumbering reaches only the collection an instance was taken out of, so a collection addressed
// through any instance of it is refused whichever of those instances goes.
TEST_CASE("a save taking instances out of a collection and of one standing under it is refused by name", "[config]")
{
    const std::filesystem::path where = authored("taken-out-nested.xml", slots_in_stations);
    const std::string before          = text_of(where);

    const std::vector<edit> changes{edit{"stations/station", "alpha", edit_kind::taken_out}, edit{"stations/station[1]/slot", "y", edit_kind::taken_out}};
    const expected<void, error> saved = save(nested(), resolve(where, scratch()), changes);
    REQUIRE_FALSE(saved.has_value());

    CHECK(saved.error().message.find("'stations/station[1]/slot'") != std::string::npos);
    CHECK(saved.error().message.find("'stations/station'") != std::string::npos);
    CHECK(text_of(where) == before);
}

// A removal stays a removal through every stage that routes edits: it is never compared as a leaf's
// value, even where its key spells one the document carries at that very text.
TEST_CASE("a removal handed through every stage that receives edits reaches the save as a removal", "[config]")
{
    const std::filesystem::path where       = authored("taken-out-routed.xml", hand_written);
    const expected<document, error> carried = load(described(), resolve(where, scratch()));
    REQUIRE(carried.has_value());

    const std::array<edit, 2> removals{edit{"stations/station", "alpha", edit_kind::taken_out}, edit{"window/title", "Overview", edit_kind::taken_out}};
    const std::vector<edit> outstanding = unsaved_edits(carried.value(), removals);
    REQUIRE(outstanding.size() == 2u);
    for(const edit &one : outstanding)
        CHECK(one.kind == edit_kind::taken_out);

    const offering standing({removals.front()});
    const std::array<const configurable *, 1> shown{&standing};
    const std::vector<edit> gathered = shown_edits(shown, carried.value());
    REQUIRE(gathered.size() == 1u);
    CHECK(gathered.front().kind == edit_kind::taken_out);
    CHECK(gathered.front().key == "stations/station");
    CHECK(gathered.front().value == "alpha");

    const expected<void, error> saved = save(described(), resolve(where, scratch()), gathered, write_policy::file_backed_only);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const expected<document, error> reloaded = load(described(), resolve(where, scratch()));
    REQUIRE(reloaded.has_value());
    CHECK(reloaded.value().identities("stations/station") == std::vector<std::string>{"beta"});
}

// A segment with no bracket names the first instance at every level of a key, so the collection a
// removal names and the key a value is written at are one place however either spells it.
TEST_CASE("a value written after a removal spelled without its ancestor's ordinal lands in the instance it named", "[config]")
{
    const std::filesystem::path where = authored("taken-out-bare-ancestor.xml", slots_under_two_stations);

    const std::vector<edit> changes{edit{"stations/station/slot", "a", edit_kind::taken_out}, edit{"stations/station[0]/slot[2]/v", "9"}};
    const expected<void, error> saved = save(nested(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());
    CHECK(text_of(where) == without_a_and_c_at_nine());
}

TEST_CASE("two instances taken out under two spellings of one collection both stand in front of a value written after them", "[config]")
{
    const std::filesystem::path where = authored("taken-out-two-spellings.xml", slots_under_two_stations);

    const std::vector<edit> changes{edit{"stations/station/slot", "a", edit_kind::taken_out}, edit{"stations/station[0]/slot", "b", edit_kind::taken_out},
                                    edit{"stations/station[0]/slot[3]/v", "9"}};
    const expected<void, error> saved = save(nested(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    std::string expected_text = with_replaced(std::string(slots_under_two_stations), "\n            <slot id=\"a\" v=\"1\"/>", "");
    expected_text             = with_replaced(expected_text, "\n            <slot id=\"b\" v=\"2\"/>", "");
    CHECK(text_of(where) == with_replaced(expected_text, "<slot id=\"d\" v=\"4\"/>", "<slot id=\"d\" v=\"9\"/>"));
}

// The value written is the declared fallback, so a misplaced write reads back as agreeing.
TEST_CASE("a value addressed to the nested instance a save takes out is refused by name under every spelling of either", "[config]")
{
    const std::array<std::string, 3> removals{"stations/station/slot", "stations/station[0]/slot", "stations[0]/station[0]/slot"};
    const std::array<std::string, 3> keys{"stations/station[0]/slot[0]/v", "stations/station/slot[0]/v", "stations/station[0]/slot/v"};
    for(std::size_t removal = 0; removal < removals.size(); ++removal)
        for(std::size_t key = 0; key < keys.size(); ++key)
        {
            INFO(removals[removal] << " beside " << keys[key]);
            const std::filesystem::path where = authored("taken-out-refused-" + std::to_string(removal) + "-" + std::to_string(key) + ".xml", slots_under_two_stations);
            const std::string before          = text_of(where);

            const std::vector<edit> changes{edit{removals[removal], "a", edit_kind::taken_out}, edit{keys[key], "0"}};
            const std::string refused = why(save(nested(), resolve(where, scratch()), changes));
            CHECK(refused.find(keys[key]) != std::string::npos);
            CHECK(refused.find("it takes out") != std::string::npos);
            CHECK(text_of(where) == before);
        }
}

TEST_CASE("every spelling of one nested removal saves the same bytes", "[config]")
{
    const std::vector<std::vector<std::string>> spellings{
            {"stations/station/slot"},
            {"stations/station[0]/slot"},
            {"stations[0]/station[0]/slot"},
            {"stations/station/slot", "stations/station[0]/slot"},
            {"stations/station/slot", "stations/station[0]/slot", "stations[0]/station[0]/slot"},
    };
    for(std::size_t set = 0; set < spellings.size(); ++set)
    {
        std::vector<edit> changes;
        std::string named;
        for(const std::string &removal : spellings[set])
        {
            changes.push_back(edit{removal, "a", edit_kind::taken_out});
            named += named.empty() ? removal : ", " + removal;
        }
        changes.push_back(edit{"stations/station[0]/slot[2]/v", "9"});

        INFO(named);
        const std::filesystem::path where = authored("taken-out-spelled-" + std::to_string(set) + ".xml", slots_under_two_stations);
        const expected<void, error> saved = save(nested(), resolve(where, scratch()), changes);
        INFO(why(saved));
        CHECK(saved.has_value());
        CHECK(text_of(where) == without_a_and_c_at_nine());
    }
}

TEST_CASE("one nested instance named for removal under two spellings goes once and its sibling stays", "[config]")
{
    const std::filesystem::path where = alone_in("removal-named-twice", slots_under_two_stations);
    const location at                 = resolve(where, where.parent_path());

    const std::vector<edit> changes{edit{"stations/station/slot", "a", edit_kind::taken_out}, edit{"stations/station[0]/slot", "a", edit_kind::taken_out}};
    std::optional<expected<void, error>> saved;
    const std::string reported = praxis::tests::reported_by([&] { saved = save(nested(), at, changes); });
    INFO(reported);
    REQUIRE(saved->has_value());

    CHECK(text_of(where) == with_replaced(std::string(slots_under_two_stations), "\n            <slot id=\"a\" v=\"1\"/>", ""));
    REQUIRE(occurrences(reported, "[info]") == 1);
    CHECK(reported.find("1 instance(s) taken out") != std::string::npos);
}

TEST_CASE("a removal whose key names one instance rather than a collection is refused by name", "[config]")
{
    const std::filesystem::path where = authored("taken-out-instance-key.xml", hand_written);
    const std::string before          = text_of(where);

    const std::vector<edit> one{edit{"stations/station[0]", "alpha", edit_kind::taken_out}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE_FALSE(saved.has_value());

    CHECK(saved.error().code == error_code::unlocatable_key);
    CHECK(saved.error().message.find("stations/station[0]") != std::string::npos);
    CHECK(text_of(where) == before);
}

TEST_CASE("an instance taken out of a document whose lines end in a carriage return and a line feed leaves every line ending that way", "[config]")
{
    const std::string before          = with_carriage_returns(hand_written);
    const std::filesystem::path where = authored("taken-out-crlf.xml", before);

    const std::vector<edit> one{edit{"stations/station", "alpha", edit_kind::taken_out}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string after = text_of(where);
    CHECK(after == with_replaced(before, "\r\n        <station name=\"alpha\"><panel scale=\"1.5\"/></station>", ""));
    CHECK(occurrences(after, "\r\r") == 0u);
    CHECK(occurrences(after, "\r\n") == occurrences(after, "\n"));
}

TEST_CASE("an instance sharing its line with a sibling goes alone and the sibling keeps its place", "[config]")
{
    const std::filesystem::path where = authored("taken-out-shared-line.xml", stations_on_one_line);

    const std::vector<edit> one{edit{"stations/station", "alpha", edit_kind::taken_out}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());
    CHECK(text_of(where) == with_replaced(std::string(stations_on_one_line), "<station name=\"alpha\"><panel scale=\"1.5\"/></station>", ""));
}

TEST_CASE("an instance alone on its line takes the blanks after it with the line", "[config]")
{
    const std::filesystem::path where = authored("taken-out-trailing-blank.xml", a_trailing_blank);

    const std::vector<edit> one{edit{"stations/station", "alpha", edit_kind::taken_out}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());
    CHECK(text_of(where) == with_replaced(std::string(a_trailing_blank), "\n        <station name=\"alpha\"><panel scale=\"1.5\"/></station>  ", ""));
}

TEST_CASE("a removal from one instance's collection leaves the same identity under another instance carried and its ordinals alone", "[config]")
{
    const std::filesystem::path where = authored("taken-out-other-parent.xml", one_identity_under_two_stations);

    const std::vector<edit> changes{edit{"stations/station/slot", "x", edit_kind::taken_out}, edit{"stations/station[1]/slot[0]/v", "7"}};
    const expected<void, error> saved = save(nested(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string without_x = with_replaced(std::string(one_identity_under_two_stations), "\n            <slot id=\"x\" v=\"1\"/>", "");
    CHECK(text_of(where) == with_replaced(without_x, "<slot id=\"x\" v=\"3\"/>", "<slot id=\"x\" v=\"7\"/>"));
}

TEST_CASE("a removal spelled with its ancestor's ordinal is read back under that ancestor", "[config]")
{
    const std::filesystem::path where = authored("taken-out-bracketed-ancestor.xml", one_identity_under_two_stations);

    const std::vector<edit> changes{edit{"stations/station[1]/slot", "x", edit_kind::taken_out}, edit{"stations/station[0]/slot[1]/v", "8"}};
    const expected<void, error> saved = save(nested(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const std::string without_x = with_replaced(std::string(one_identity_under_two_stations), "\n            <slot id=\"x\" v=\"3\"/>", "");
    CHECK(text_of(where) == with_replaced(without_x, "<slot id=\"y\" v=\"2\"/>", "<slot id=\"y\" v=\"8\"/>"));
}
