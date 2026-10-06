#include "engine.h"

#include "praxis/config/writer.h"

#include <array>
#include <cmath>
#include <cerrno>
#include <string>
#include <cstddef>
#include <cstdlib>
#include <charconv>
#include <optional>
#include <algorithm>
#include <functional>
#include <string_view>

#include <locale.h>
// macOS declares newlocale and strtod_l here, for the headers included before it.
#if __has_include(<xlocale.h>)
    #include <xlocale.h>
#endif

namespace praxis::config {
namespace {

std::string_view trimmed(std::string_view text)
{
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if(first == std::string_view::npos)
        return std::string_view();
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

char lowered(char one)
{
    return one >= 'A' && one <= 'Z' ? static_cast<char>(one - 'A' + 'a') : one;
}

bool spelled_as(std::string_view text, std::string_view lower)
{
    return std::ranges::equal(text, lower, std::ranges::equal_to{}, lowered);
}

bool n_char(char one)
{
    const char low = lowered(one);
    return (low >= 'a' && low <= 'z') || (one >= '0' && one <= '9') || one == '_';
}

bool numeral(std::string_view text)
{
    return !text.empty() && text.find_first_not_of("0123456789+-.eE") == std::string_view::npos;
}

std::string_view unsigned_plus(std::string_view text)
{
    if(text.size() > 1 && text.front() == '+' && text[1] != '+' && text[1] != '-')
        text.remove_prefix(1);
    return text;
}

#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
std::optional<double> parsed(std::string_view text)
{
    double value                      = 0.0;
    const std::from_chars_result done = std::from_chars(text.data(), text.data() + text.size(), value);
    if(done.ec != std::errc() || done.ptr != text.data() + text.size())
        return std::nullopt;
    return value;
}
#else
std::optional<double> parsed(std::string_view text)
{
    static const locale_t classic = newlocale(LC_ALL_MASK, "C", locale_t{});
    if(classic == locale_t{})
        return std::nullopt;

    const std::string held(text);
    char *end          = nullptr;
    errno              = 0;
    const double value = strtod_l(held.c_str(), &end, classic);
    if(end != held.c_str() + held.size() || (errno == ERANGE && (value == 0.0 || std::isinf(value))))
        return std::nullopt;
    return value;
}
#endif

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

// Read in the document's own decimal grammar whatever the process's locale; a text that underflows
// to zero, overflows or names no finite number does not read.
std::optional<double> as_real(std::string_view text)
{
    const std::string_view value = trimmed(text);
    if(!numeral(value))
        return std::nullopt;

    const std::optional<double> read = parsed(unsigned_plus(value));
    if(!read || !std::isfinite(*read))
        return std::nullopt;
    return read;
}

bool non_finite(std::string_view text)
{
    std::string_view value = trimmed(text);
    if(value.starts_with('+') || value.starts_with('-'))
        value.remove_prefix(1);
    if(spelled_as(value, "inf") || spelled_as(value, "infinity") || spelled_as(value, "nan"))
        return true;
    if(value.size() < 5 || !spelled_as(value.substr(0, 4), "nan(") || !value.ends_with(')'))
        return false;
    return std::ranges::all_of(value.substr(4, value.size() - 5), n_char);
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

std::string exact_text(double value)
{
    std::array<char, 40> digits{};
    const std::to_chars_result printed = std::to_chars(digits.data(), digits.data() + digits.size(), value);
    return printed.ec == std::errc() ? std::string(digits.data(), printed.ptr) : std::string();
}

}
