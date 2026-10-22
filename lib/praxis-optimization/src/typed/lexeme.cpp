#include "parse.h"

#include <array>
#include <cmath>
#include <cerrno>
#include <string>
#include <cstddef>
#include <cstdlib>
#include <numbers>
#include <utility>
#include <charconv>
#include <optional>
#include <string_view>

#include <locale.h>
// macOS declares newlocale and strtod_l here, for the headers included before it.
#if __has_include(<xlocale.h>)
    #include <xlocale.h>
#endif

namespace praxis::optimization::typed {
namespace {

struct named_constant
{
    double value;
    std::string_view spelling;
};

constexpr std::array constants{named_constant{std::numbers::pi, "pi"}, named_constant{std::numbers::e, "e"}};

template<typename Table, typename Spelling>
std::pair<std::size_t, std::size_t> longest(const Table &table, std::string_view rest, Spelling spelling)
{
    std::pair<std::size_t, std::size_t> found{table.size(), 0};
    for(std::size_t i = 0; i < table.size(); ++i)
        if(const std::string_view one = spelling(table[i]); rest.starts_with(one) && one.size() > found.second)
            found = {i, one.size()};
    return found;
}

}

bool digit(char one)
{
    return one >= '0' && one <= '9';
}

bool lowercase(char one)
{
    return one >= 'a' && one <= 'z';
}

// A letter run takes the longest function or constant name starting where it stands before a single
// letter is read, so `spin` reads s*pi*n.
node name_at(std::string_view text, std::size_t at, std::size_t dimension)
{
    const std::string_view rest    = text.substr(at);
    const auto [function, spelled] = longest(function_names, rest, [](std::string_view one) { return one; });
    const auto [constant, length]  = longest(constants, rest, [](const named_constant &one) { return one.spelling; });
    if(spelled > length)
        return node{'\0', 0.0, node_kind::function, 0, 0, at, spelled, static_cast<named_function>(function)};
    if(length > 0)
        return node{'\0', constants[constant].value, node_kind::constant, 0, 0, at, length, named_function::sin};
    const bool variable = rest.front() == 'x' || (rest.front() == 'y' && dimension == 2);
    return node{rest.front(), 0.0, variable ? node_kind::variable : node_kind::parameter, 0, 0, at, 1, named_function::sin};
}

std::size_t number_length(std::string_view rest)
{
    std::size_t length = 0;
    std::size_t digits = 0;
    bool point         = false;
    for(; length < rest.size(); ++length)
    {
        if(digit(rest[length]))
            ++digits;
        else if(rest[length] == '.' && !point)
            point = true;
        else
            break;
    }
    return digits == 0 ? 0 : length;
}

#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
std::optional<double> decimal(std::string_view lexeme)
{
    double value                      = 0.0;
    const std::from_chars_result done = std::from_chars(lexeme.data(), lexeme.data() + lexeme.size(), value);
    if(done.ec != std::errc() || done.ptr != lexeme.data() + lexeme.size())
        return std::nullopt;
    return value;
}
#else
std::optional<double> decimal(std::string_view lexeme)
{
    static const locale_t classic = newlocale(LC_ALL_MASK, "C", locale_t{});
    if(classic == locale_t{})
        return std::nullopt;

    const std::string held(lexeme);
    char *end          = nullptr;
    errno              = 0;
    const double value = strtod_l(held.c_str(), &end, classic);
    if(end != held.c_str() + held.size() || (errno == ERANGE && (value == 0.0 || std::isinf(value))))
        return std::nullopt;
    return value;
}
#endif

}
