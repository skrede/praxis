#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_EXPRESSION_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_EXPRESSION_H

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::optimization::typed {

enum class node_kind : std::uint8_t
{
    number,
    variable,
    parameter,
    constant,
    negate,
    add,
    subtract,
    multiply,
    divide,
    power,
    function
};

enum class named_function : std::uint8_t
{
    sin,
    cos,
    tan,
    exp,
    log,
    sqrt
};

inline constexpr std::array<std::string_view, 6> function_names{"sin", "cos", "tan", "exp", "log", "sqrt"};

// One node of a parsed text. `left` is the operand of a negation or a function and the left operand of
// a binary node, `right` its right operand; `offset` and `length` span the bytes the node was read
// from: a leaf's lexeme, a function's name, an operator's symbol, and nothing for an implicit product.
struct node
{
    char letter;
    double value;
    node_kind kind;
    std::size_t left;
    std::size_t right;
    std::size_t offset;
    std::size_t length;
    named_function function;
};

// The nodes are in post-order, every operand before the node reading it and the root last;
// `parameters` holds each parameter letter the text uses once, sorted.
struct expression
{
    std::string text;
    std::size_t dimension;
    std::vector<node> nodes;
    std::string parameters;
};

// The longest text, in bytes, that is read.
inline constexpr std::size_t expression_capacity = 512;

}

#endif
