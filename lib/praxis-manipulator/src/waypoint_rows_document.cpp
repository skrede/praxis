#include "configuration_keys.h"
#include "waypoint_rows_document.h"

#include <span>
#include <locale>
#include <string>
#include <vector>
#include <cstddef>
#include <sstream>
#include <algorithm>
#include <string_view>

namespace praxis::manipulator::waypoint_rows {

namespace {

std::vector<double> values_of(const std::string &text)
{
    std::vector<double> taken;
    for(const carried_number &number : numbers_of(text))
        taken.push_back(number.value);

    return taken;
}

// A row carrying no text at all is a row a shorter list left behind, so it is passed over rather
// than read as a row of no numbers.
std::string row_text(const config::document &values, const std::string &rows, const std::string &identity, std::string_view leaf)
{
    const expected<std::string, config::error> key = values.key(rows, identity, leaf);
    if(!key)
        return std::string();

    const expected<std::string, config::error> text = values.text(*key);

    return text ? *text : std::string();
}

// The ordinals of the instances carrying an identity, which are the rows a list is written into.
std::vector<std::size_t> indexed_in(const std::vector<std::string> &present)
{
    std::vector<std::size_t> indexed;
    for(std::size_t ordinal = 0; ordinal < present.size(); ++ordinal)
        if(!present[ordinal].empty())
            indexed.push_back(ordinal);

    return indexed;
}

std::string instance(const std::string &held, std::size_t ordinal)
{
    return held + "[" + std::to_string(ordinal) + "]";
}

}

void declare(config::declaration &shape, std::string_view at, std::string_view leaf)
{
    const std::string rows = keys::under(at, leaf_names::row);

    shape.group(std::string(at));
    shape.collection(rows, std::string(leaf_names::index));
    shape.field(keys::under(rows, leaf), config::field_kind::text, std::string());
}

std::vector<carried_row> read(const config::document &values, std::string_view at, std::string_view leaf)
{
    const std::string rows = keys::under(at, leaf_names::row);

    std::vector<carried_row> carried;
    for(const std::string &identity : values.identities(rows))
    {
        const std::string text = row_text(values, rows, identity, leaf);
        if(!text.empty())
            carried.push_back(carried_row{identity, values_of(text)});
    }

    return carried;
}

// The stream is imbued with the classic locale because a decimal point is a property of the
// document's grammar rather than of whatever locale the process happens to run under.
std::vector<carried_number> numbers_of(std::string_view text)
{
    std::istringstream reader{std::string(text)};
    reader.imbue(std::locale::classic());

    std::vector<carried_number> taken;
    for(double value = 0.0; !(reader >> std::ws).eof();)
    {
        const auto begun = static_cast<std::size_t>(reader.tellg());
        if(!(reader >> value))
            break;

        const std::size_t ended = reader.eof() ? text.size() : static_cast<std::size_t>(reader.tellg());
        taken.push_back(carried_number{std::string(text.substr(begun, ended - begun)), value});
    }

    return reader.eof() ? taken : std::vector<carried_number>();
}

std::string joined(std::span<const std::string> numbers)
{
    std::string text;
    for(std::size_t number = 0; number < numbers.size(); ++number)
    {
        if(number != 0u)
            text += ' ';
        text += numbers[number];
    }

    return text;
}

std::vector<config::edit> write(const config::document &values, std::span<const std::string> rows, std::string_view at, std::string_view leaf)
{
    const std::string held                 = keys::under(at, leaf_names::row);
    const std::vector<std::string> present = values.identities(held);
    const std::vector<std::size_t> indexed = indexed_in(present);

    std::vector<config::edit> changes;
    for(std::size_t row = 0; row < std::max(indexed.size(), rows.size()); ++row)
    {
        const bool carried      = row < indexed.size();
        const std::string where = instance(held, carried ? indexed[row] : present.size() + row - indexed.size());
        if(!carried)
            changes.push_back(config::edit{keys::under(where, leaf_names::index), std::to_string(row + 1u)});

        changes.push_back(config::edit{keys::under(where, leaf), row < rows.size() ? rows[row] : std::string()});
    }

    return changes;
}

std::vector<std::string> addressed_texts(const config::document &values, std::string_view at, std::string_view leaf)
{
    const std::string held = keys::under(at, leaf_names::row);

    std::vector<std::string> texts;
    for(const std::size_t ordinal : indexed_in(values.identities(held)))
    {
        const expected<std::string, config::error> text = values.text(keys::under(instance(held, ordinal), leaf));
        texts.push_back(text ? *text : std::string());
    }

    return texts;
}

}
