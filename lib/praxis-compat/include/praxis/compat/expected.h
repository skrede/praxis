#ifndef HPP_GUARD_PRAXIS_COMPAT_EXPECTED_H
#define HPP_GUARD_PRAXIS_COMPAT_EXPECTED_H

#include "praxis/compat/detail/expected.h"

#include <utility>
#include <version>

#if defined(__cpp_lib_expected)
    #include <expected>
#endif

namespace praxis {

// A class template with a deduction guide rather than an alias template: alias-template class
// template argument deduction (P1814) is GCC-only, so unexpected(err) written through an alias
// fails to compile on Clang.
// MSVC's <eh.h> declares a global unexpected(), so a translation unit carrying a using-directive
// for this namespace must qualify the call.
template<typename E>
class unexpected
{
public:
    explicit constexpr unexpected(E err)
            : m_error(std::move(err))
    {
    }

#if defined(__cpp_lib_expected)
    explicit constexpr unexpected(const std::unexpected<E> &other)
            : m_error(other.error())
    {
    }

    explicit constexpr unexpected(std::unexpected<E> &&other)
            : m_error(std::move(other).error())
    {
    }

    explicit constexpr operator std::unexpected<E>() const &
    {
        return std::unexpected<E>(m_error);
    }

    explicit constexpr operator std::unexpected<E>() &&
    {
        return std::unexpected<E>(std::move(m_error));
    }
#endif

    constexpr E &error() & noexcept
    {
        return m_error;
    }

    constexpr const E &error() const & noexcept
    {
        return m_error;
    }

    constexpr E &&error() && noexcept
    {
        return std::move(m_error);
    }

    constexpr const E &&error() const && noexcept
    {
        return std::move(m_error);
    }

private:
    E m_error;
};

template<typename E>
unexpected(E) -> unexpected<E>;

using detail::expected;

}

#endif
