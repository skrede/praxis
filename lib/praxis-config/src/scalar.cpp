#include "engine.h"

#include <ios>
#include <string>
#include <locale>
#include <cstddef>
#include <sstream>
#include <charconv>
#include <optional>
#include <string_view>

namespace praxis::config {
namespace {

std::string_view trimmed(std::string_view text)
{
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if(first == std::string_view::npos)
        return std::string_view();
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

}

std::optional<bool> as_flag(std::string_view text)
{
    const std::string_view value = trimmed(text);
    if(value == "true" || value == "1")
        return true;
    if(value == "false" || value == "0")
        return false;
    return std::nullopt;
}

// The stream is imbued with the classic locale because a decimal point is a property of the
// document's grammar rather than of whatever locale the process happens to run under.
std::optional<double> as_real(std::string_view text)
{
    std::istringstream reader(std::string(trimmed(text)));
    reader.imbue(std::locale::classic());
    double value = 0.0;
    reader >> value;
    if(reader.fail() || !reader.eof())
        return std::nullopt;
    return value;
}

std::optional<std::int64_t> as_integer(std::string_view text)
{
    const std::string_view value      = trimmed(text);
    std::int64_t parsed               = 0;
    const std::from_chars_result done = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if(done.ec != std::errc() || done.ptr != value.data() + value.size())
        return std::nullopt;
    return parsed;
}

}
