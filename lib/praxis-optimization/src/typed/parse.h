#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_PARSE_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_PARSE_H

#include "expression.h"

#include "praxis/compat/expected.h"

#include <string>
#include <cstddef>
#include <string_view>

namespace praxis::optimization::typed {

struct parse_failure
{
    std::size_t offset;
    std::string expected;
};

// In one dimension x is the variable, in two x and y; `pi` and `e` are constants and every other
// lowercase letter is a parameter. A number has no exponent part, so one directly followed by `e` and a
// digit, signed or not, is refused rather than read as a product with e.
expected<expression, parse_failure> parsed(std::string_view text, std::size_t dimension);

std::string echoed(const expression &read);

bool digit(char one);
bool lowercase(char one);

// The length of the decimal lexeme `rest` starts with: digits with at most one '.', no exponent part;
// zero where it holds no digit.
std::size_t number_length(std::string_view rest);

// The function, constant or single letter at the lowercase letter `at`, as a leaf spanning its spelling.
node name_at(std::string_view text, std::size_t at, std::size_t dimension);

}

#endif
