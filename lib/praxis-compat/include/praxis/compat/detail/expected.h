#ifndef HPP_GUARD_PRAXIS_COMPAT_DETAIL_EXPECTED_H
#define HPP_GUARD_PRAXIS_COMPAT_DETAIL_EXPECTED_H

#include <memory>
#include <cstdlib>
#include <utility>
#include <version>
#include <exception>
#include <type_traits>

#if defined(__cpp_lib_expected)
    #include <expected>
#endif

#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    #define PRAXIS_HAS_EXCEPTIONS 1
#else
    #define PRAXIS_HAS_EXCEPTIONS 0
#endif

namespace praxis {

template<typename E>
class unexpected;

}

namespace praxis::detail {

#if PRAXIS_HAS_EXCEPTIONS
struct bad_expected_access : std::exception
{
    const char *what() const noexcept override
    {
        return "bad praxis::expected access";
    }
};
#endif

[[noreturn]] inline void on_bad_expected_access()
{
#if PRAXIS_HAS_EXCEPTIONS
    throw bad_expected_access{};
#else
    std::abort();
#endif
}

template<typename T, typename E>
class [[nodiscard]] expected
{
public:
    constexpr expected(T val)
            : m_has_value(true)
    {
        std::construct_at(std::addressof(m_value), std::move(val));
    }

    template<typename U, std::enable_if_t<std::is_constructible_v<T, U> && !std::is_same_v<std::remove_cvref_t<U>, expected>, int> = 0>
    constexpr expected(U &&val)
            : m_has_value(true)
    {
        std::construct_at(std::addressof(m_value), T(std::forward<U>(val)));
    }

    constexpr expected(unexpected<E> err)
            : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), std::move(err).error());
    }

    constexpr expected(const expected &other)
            : m_has_value(other.m_has_value)
    {
        if(m_has_value)
            std::construct_at(std::addressof(m_value), other.m_value);
        else
            std::construct_at(std::addressof(m_error), other.m_error);
    }

    constexpr expected(expected &&other) noexcept(std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_constructible_v<E>)
            : m_has_value(other.m_has_value)
    {
        if(m_has_value)
            std::construct_at(std::addressof(m_value), std::move(other.m_value));
        else
            std::construct_at(std::addressof(m_error), std::move(other.m_error));
    }

#if defined(__cpp_lib_expected)
    explicit constexpr expected(const std::expected<T, E> &other)
            : m_has_value(other.has_value())
    {
        if(m_has_value)
            std::construct_at(std::addressof(m_value), *other);
        else
            std::construct_at(std::addressof(m_error), other.error());
    }

    explicit constexpr expected(std::expected<T, E> &&other)
            : m_has_value(other.has_value())
    {
        if(m_has_value)
            std::construct_at(std::addressof(m_value), *std::move(other));
        else
            std::construct_at(std::addressof(m_error), std::move(other).error());
    }

    explicit constexpr operator std::expected<T, E>() const &
    {
        if(m_has_value)
            return std::expected<T, E>(m_value);
        return std::expected<T, E>(std::unexpect, m_error);
    }

    explicit constexpr operator std::expected<T, E>() &&
    {
        if(m_has_value)
            return std::expected<T, E>(std::move(m_value));
        return std::expected<T, E>(std::unexpect, std::move(m_error));
    }
#endif

    constexpr expected &operator=(const expected &other)
        requires(std::is_copy_assignable_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<E> && std::is_copy_constructible_v<E> &&
                 (std::is_nothrow_move_constructible_v<T> || std::is_nothrow_move_constructible_v<E>))
    {
        if(m_has_value && other.m_has_value)
            m_value = other.m_value;
        else if(!m_has_value && !other.m_has_value)
            m_error = other.m_error;
        else if(other.m_has_value)
            reinit_as_value(other.m_value);
        else
            reinit_as_error(other.m_error);
        return *this;
    }

    constexpr expected &operator=(expected &&other) noexcept(std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_assignable_v<T> &&
                                                             std::is_nothrow_move_constructible_v<E> && std::is_nothrow_move_assignable_v<E>)
        requires(std::is_move_assignable_v<T> && std::is_move_constructible_v<T> && std::is_move_assignable_v<E> && std::is_move_constructible_v<E> &&
                 (std::is_nothrow_move_constructible_v<T> || std::is_nothrow_move_constructible_v<E>))
    {
        if(m_has_value && other.m_has_value)
            m_value = std::move(other.m_value);
        else if(!m_has_value && !other.m_has_value)
            m_error = std::move(other.m_error);
        else if(other.m_has_value)
            reinit_as_value(std::move(other.m_value));
        else
            reinit_as_error(std::move(other.m_error));
        return *this;
    }

    constexpr ~expected()
    {
        if constexpr(!std::is_trivially_destructible_v<T>)
        {
            if(m_has_value)
                std::destroy_at(std::addressof(m_value));
        }
        if constexpr(!std::is_trivially_destructible_v<E>)
        {
            if(!m_has_value)
                std::destroy_at(std::addressof(m_error));
        }
    }

    constexpr explicit operator bool() const noexcept
    {
        return m_has_value;
    }

    constexpr bool has_value() const noexcept
    {
        return m_has_value;
    }

    constexpr T &operator*() & noexcept
    {
        return m_value;
    }

    constexpr const T &operator*() const & noexcept
    {
        return m_value;
    }

    constexpr T &&operator*() && noexcept
    {
        return std::move(m_value);
    }

    constexpr const T &&operator*() const && noexcept
    {
        return std::move(m_value);
    }

    constexpr T *operator->() noexcept
    {
        return std::addressof(m_value);
    }

    constexpr const T *operator->() const noexcept
    {
        return std::addressof(m_value);
    }

    constexpr T &value() &
    {
        if(!m_has_value)
            on_bad_expected_access();
        return m_value;
    }

    constexpr const T &value() const &
    {
        if(!m_has_value)
            on_bad_expected_access();
        return m_value;
    }

    constexpr T &&value() &&
    {
        if(!m_has_value)
            on_bad_expected_access();
        return std::move(m_value);
    }

    constexpr const T &&value() const &&
    {
        if(!m_has_value)
            on_bad_expected_access();
        return std::move(m_value);
    }

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

    template<typename U>
    constexpr T value_or(U &&fallback) const &
    {
        return m_has_value ? m_value : static_cast<T>(std::forward<U>(fallback));
    }

    template<typename U>
    constexpr T value_or(U &&fallback) &&
    {
        return m_has_value ? std::move(m_value) : static_cast<T>(std::forward<U>(fallback));
    }

private:
    bool m_has_value;
    union
    {
        T m_value;
        E m_error;
    };

    // Ends the lifetime of *old_member and constructs *new_member from args in the same storage,
    // through whichever of three branches cannot leave the union holding a member whose lifetime
    // already ended. m_has_value is left alone; the caller flips it only after this returns, so an
    // unwound assignment cannot leave the flag disagreeing with the union's real occupant.
    template<typename NewT, typename OldT, typename... Args>
    static constexpr void reinit_member(NewT *new_member, OldT *old_member, Args &&...args)
    {
        if constexpr(std::is_nothrow_constructible_v<NewT, Args...>)
        {
            if constexpr(!std::is_trivially_destructible_v<OldT>)
                std::destroy_at(old_member);
            std::construct_at(new_member, std::forward<Args>(args)...);
        }
        else if constexpr(std::is_nothrow_move_constructible_v<NewT>)
        {
            NewT staged(std::forward<Args>(args)...);
            if constexpr(!std::is_trivially_destructible_v<OldT>)
                std::destroy_at(old_member);
            std::construct_at(new_member, std::move(staged));
        }
        else
        {
            static_assert(std::is_nothrow_move_constructible_v<OldT>, "the assignment constraints guarantee the staged member moves back into the union without throwing");
            OldT staged(std::move(*old_member));
            if constexpr(!std::is_trivially_destructible_v<OldT>)
                std::destroy_at(old_member);
#if PRAXIS_HAS_EXCEPTIONS
            try
            {
                std::construct_at(new_member, std::forward<Args>(args)...);
            }
            catch(...)
            {
                std::construct_at(old_member, std::move(staged));
                throw;
            }
#else
            std::construct_at(new_member, std::forward<Args>(args)...);
#endif
        }
    }

    template<typename Arg>
    constexpr void reinit_as_value(Arg &&arg)
    {
        reinit_member(std::addressof(m_value), std::addressof(m_error), std::forward<Arg>(arg));
        m_has_value = true;
    }

    template<typename Arg>
    constexpr void reinit_as_error(Arg &&arg)
    {
        reinit_member(std::addressof(m_error), std::addressof(m_value), std::forward<Arg>(arg));
        m_has_value = false;
    }
};

template<typename E>
class [[nodiscard]] expected<void, E>
{
public:
    constexpr expected() noexcept
            : m_has_value(true)
    {
    }

    constexpr expected(unexpected<E> err)
            : m_has_value(false)
    {
        std::construct_at(std::addressof(m_error), std::move(err).error());
    }

    constexpr expected(const expected &other)
            : m_has_value(other.m_has_value)
    {
        if(!m_has_value)
            std::construct_at(std::addressof(m_error), other.m_error);
    }

    constexpr expected(expected &&other) noexcept(std::is_nothrow_move_constructible_v<E>)
            : m_has_value(other.m_has_value)
    {
        if(!m_has_value)
            std::construct_at(std::addressof(m_error), std::move(other.m_error));
    }

#if defined(__cpp_lib_expected)
    explicit constexpr expected(const std::expected<void, E> &other)
            : m_has_value(other.has_value())
    {
        if(!m_has_value)
            std::construct_at(std::addressof(m_error), other.error());
    }

    explicit constexpr expected(std::expected<void, E> &&other)
            : m_has_value(other.has_value())
    {
        if(!m_has_value)
            std::construct_at(std::addressof(m_error), std::move(other).error());
    }

    explicit constexpr operator std::expected<void, E>() const &
    {
        if(m_has_value)
            return std::expected<void, E>();
        return std::expected<void, E>(std::unexpect, m_error);
    }

    explicit constexpr operator std::expected<void, E>() &&
    {
        if(m_has_value)
            return std::expected<void, E>();
        return std::expected<void, E>(std::unexpect, std::move(m_error));
    }
#endif

    constexpr expected &operator=(const expected &other)
    {
        if(m_has_value == other.m_has_value)
        {
            if(!m_has_value)
                m_error = other.m_error;
        }
        else if(other.m_has_value)
            destroy_error();
        else
            std::construct_at(std::addressof(m_error), other.m_error);
        m_has_value = other.m_has_value;
        return *this;
    }

    constexpr expected &operator=(expected &&other) noexcept(std::is_nothrow_move_constructible_v<E> && std::is_nothrow_move_assignable_v<E>)
    {
        if(m_has_value == other.m_has_value)
        {
            if(!m_has_value)
                m_error = std::move(other.m_error);
        }
        else if(other.m_has_value)
            destroy_error();
        else
            std::construct_at(std::addressof(m_error), std::move(other.m_error));
        m_has_value = other.m_has_value;
        return *this;
    }

    constexpr ~expected()
    {
        destroy_error();
    }

    constexpr explicit operator bool() const noexcept
    {
        return m_has_value;
    }

    constexpr bool has_value() const noexcept
    {
        return m_has_value;
    }

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
    bool m_has_value;
    union
    {
        E m_error;
    };

    constexpr void destroy_error()
    {
        if constexpr(!std::is_trivially_destructible_v<E>)
        {
            if(!m_has_value)
                std::destroy_at(std::addressof(m_error));
        }
    }
};

}

#endif
