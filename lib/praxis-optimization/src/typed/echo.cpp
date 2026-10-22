#include "parse.h"

#include <string>
#include <cstddef>
#include <string_view>

namespace praxis::optimization::typed {
namespace {

bool primary(node_kind kind)
{
    return kind == node_kind::number || kind == node_kind::variable || kind == node_kind::parameter || kind == node_kind::constant || kind == node_kind::function;
}

bool sum(node_kind kind)
{
    return kind == node_kind::add || kind == node_kind::subtract;
}

bool product(node_kind kind)
{
    return kind == node_kind::multiply || kind == node_kind::divide;
}

// Whether an operand is parenthesized: wherever reading its echo back could give another tree, and
// around a negation that is an operand of `*`, `/`, `^` or the right of `-`.
bool wrapped(node_kind parent, node_kind child, bool right)
{
    if(parent == node_kind::negate || parent == node_kind::power)
        return !primary(child);
    if(product(parent))
        return sum(child) || child == node_kind::negate || (right ? product(child) : child == node_kind::divide);
    return right && (sum(child) || (parent == node_kind::subtract && child == node_kind::negate));
}

std::string_view symbol(node_kind kind)
{
    switch(kind)
    {
        case node_kind::add:
            return " + ";
        case node_kind::subtract:
            return " - ";
        case node_kind::multiply:
            return "*";
        case node_kind::divide:
            return "/";
        default:
            return "^";
    }
}

std::string echo_at(const expression &read, std::size_t at);

std::string operand(const expression &read, std::size_t parent, std::size_t child, bool right)
{
    std::string text = echo_at(read, child);
    return wrapped(read.nodes[parent].kind, read.nodes[child].kind, right) ? "(" + text + ")" : text;
}

std::string echo_at(const expression &read, std::size_t at)
{
    const node &one = read.nodes[at];
    switch(one.kind)
    {
        case node_kind::number:
        case node_kind::constant:
            return read.text.substr(one.offset, one.length);
        case node_kind::variable:
        case node_kind::parameter:
            return std::string(1, one.letter);
        case node_kind::negate:
            return "-" + operand(read, at, one.left, false);
        case node_kind::function:
            return std::string(function_names[static_cast<std::size_t>(one.function)]) + "(" + echo_at(read, one.left) + ")";
        default:
            return operand(read, at, one.left, false) + std::string(symbol(one.kind)) + operand(read, at, one.right, true);
    }
}

}

std::string echoed(const expression &read)
{
    return read.nodes.empty() ? std::string() : echo_at(read, read.nodes.size() - 1);
}

}
