#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_ANSWERED_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_ANSWERED_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

namespace praxis::tests {

// The reference is valid only for the enclosing full expression, which is where a result handed in
// as a temporary lives.
template<typename T>
const T &answered(const expected<T, refusal> &result)
{
    REQUIRE(result.has_value());

    return *result;
}

}

#endif
