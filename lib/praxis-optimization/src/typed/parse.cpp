#include "parse.h"

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::optimization::typed {
namespace {

constexpr std::string_view operators = " .+-*/^()";

struct cursor
{
    std::size_t at;
    std::size_t dimension;
    std::string_view text;
    std::vector<node> nodes;
    std::size_t after_number;
    std::optional<parse_failure> failure;
};

bool starts_primary(char one)
{
    return digit(one) || lowercase(one) || one == '.' || one == '(';
}

bool exponent_like(std::string_view rest)
{
    const std::size_t sign = rest.size() > 1 && (rest[1] == '+' || rest[1] == '-') ? 1 : 0;
    return rest.size() > sign + 1 && rest.front() == 'e' && digit(rest[sign + 1]);
}

std::size_t skipped(std::string_view text, std::size_t from)
{
    return std::min(text.find_first_not_of(' ', from), text.size());
}

char peek(cursor &c)
{
    c.at = skipped(c.text, c.at);
    return c.at < c.text.size() ? c.text[c.at] : '\0';
}

// A byte the grammar never reads is the fault wherever the reading stops on it.
bool fail(cursor &c, std::size_t at, std::string expected)
{
    if(at < c.text.size() && !digit(c.text[at]) && !lowercase(c.text[at]) && operators.find(c.text[at]) == std::string_view::npos)
        expected = c.text[at] >= 'A' && c.text[at] <= 'Z' ? "a lowercase letter" : "a digit, a letter, an operator or a parenthesis";
    c.failure = parse_failure{at, std::move(expected)};
    return false;
}

bool push(cursor &c, node_kind kind, std::size_t left, std::size_t offset, std::size_t length)
{
    c.nodes.push_back(node{'\0', 0.0, kind, left, c.nodes.size() - 1, offset, length, named_function::sin});
    return true;
}

bool read_sum(cursor &c);
bool read_signed(cursor &c);

bool read_closed(cursor &c)
{
    if(!read_sum(c))
        return false;
    if(peek(c) != ')')
        return fail(c, c.at, "')'");
    ++c.at;
    return true;
}

bool read_letters(cursor &c)
{
    node name = name_at(c.text, c.at, c.dimension);
    c.at += name.length;
    if(name.kind == node_kind::function)
    {
        if(peek(c) != '(')
            return fail(c, c.at, "'(' after " + std::string(function_names[static_cast<std::size_t>(name.function)]));
        ++c.at;
        if(!read_closed(c))
            return false;
        name.left = name.right = c.nodes.size() - 1;
    }
    c.nodes.push_back(name);
    return true;
}

bool read_number(cursor &c)
{
    const std::size_t start = c.at;
    if(c.after_number != std::string_view::npos && skipped(c.text, c.after_number) == start)
        return fail(c, start, "an operator between two numbers");
    const std::size_t length          = number_length(c.text.substr(start));
    const std::optional<double> value = length == 0 ? std::nullopt : decimal(c.text.substr(start, length));
    if(!value)
        return fail(c, start, length == 0 ? "a digit" : "a number a double holds");
    if(exponent_like(c.text.substr(start + length)))
        return fail(c, start + length, "an operator between a number and e");
    c.nodes.push_back(node{'\0', *value, node_kind::number, 0, 0, start, length, named_function::sin});
    c.at = c.after_number = start + length;
    return true;
}

bool read_primary(cursor &c)
{
    const char next = peek(c);
    if(digit(next) || next == '.')
        return read_number(c);
    if(lowercase(next))
        return read_letters(c);
    if(next != '(')
        return fail(c, c.at, "an operand");
    ++c.at;
    return read_closed(c);
}

// `^` is right-associative and binds tighter than a unary minus, so -x^2 reads -(x^2) and 2^-x^2
// reads 2^(-(x^2)).
bool read_power(cursor &c)
{
    if(!read_primary(c))
        return false;
    if(peek(c) != '^')
        return true;
    const std::size_t base = c.nodes.size() - 1;
    const std::size_t op   = c.at++;
    return read_signed(c) && push(c, node_kind::power, base, op, 1);
}

bool read_signed(cursor &c)
{
    if(peek(c) != '-')
        return read_power(c);
    const std::size_t op = c.at++;
    return read_signed(c) && push(c, node_kind::negate, c.nodes.size() - 1, op, 1);
}

// A primary directly after a factor multiplies it, binding as `*` does and left to right, so 1/2x
// reads (1/2)*x and 2x^3 reads 2*(x^3).
bool read_product(cursor &c)
{
    if(!read_signed(c))
        return false;
    for(char next = peek(c); next == '*' || next == '/' || starts_primary(next); next = peek(c))
    {
        const std::size_t left = c.nodes.size() - 1;
        const std::size_t op   = c.at;
        const bool written     = next == '*' || next == '/';
        c.at += written ? 1 : 0;
        if(!(written ? read_signed(c) : read_power(c)))
            return false;
        push(c, next == '/' ? node_kind::divide : node_kind::multiply, left, op, written ? 1 : 0);
    }
    return true;
}

bool read_sum(cursor &c)
{
    if(!read_product(c))
        return false;
    for(char next = peek(c); next == '+' || next == '-'; next = peek(c))
    {
        const std::size_t left = c.nodes.size() - 1;
        const std::size_t op   = c.at++;
        if(!read_product(c))
            return false;
        push(c, next == '+' ? node_kind::add : node_kind::subtract, left, op, 1);
    }
    return true;
}

std::string parameters_of(const std::vector<node> &nodes)
{
    std::string letters;
    for(const node &one : nodes)
        if(one.kind == node_kind::parameter)
            letters.push_back(one.letter);
    std::ranges::sort(letters);
    letters.erase(std::ranges::unique(letters).begin(), letters.end());
    return letters;
}

}

expected<expression, parse_failure> parsed(std::string_view text, std::size_t dimension)
{
    if(text.size() > expression_capacity)
        return unexpected(parse_failure{expression_capacity, "the end within " + std::to_string(expression_capacity) + " bytes"});

    cursor c{0, dimension, text, {}, std::string_view::npos, std::nullopt};
    if(read_sum(c) && skipped(text, c.at) < text.size())
        fail(c, skipped(text, c.at), "an operator or the end");
    if(c.failure)
        return unexpected(std::move(*c.failure));
    std::string parameters = parameters_of(c.nodes);
    return expression{std::string(text), dimension, std::move(c.nodes), std::move(parameters)};
}

}
