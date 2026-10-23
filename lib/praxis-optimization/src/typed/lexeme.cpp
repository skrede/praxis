#include "parse.h"

#include <array>
#include <cstddef>
#include <numbers>
#include <utility>
#include <string_view>

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

bool exponent_like(std::string_view rest)
{
    const std::size_t sign = rest.size() > 1 && (rest[1] == '+' || rest[1] == '-') ? 1 : 0;
    return rest.size() > sign + 1 && rest.front() == 'e' && digit(rest[sign + 1]);
}

}
