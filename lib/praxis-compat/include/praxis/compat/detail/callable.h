#ifndef HPP_GUARD_PRAXIS_COMPAT_DETAIL_CALLABLE_H
#define HPP_GUARD_PRAXIS_COMPAT_DETAIL_CALLABLE_H

#include <new>
#include <cstddef>
#include <utility>
#include <version>
#include <type_traits>

#if defined(__cpp_lib_move_only_function)
    #include <functional>
#endif

namespace praxis::detail {

[[noreturn]] void called_without_target();

#if defined(__cpp_lib_move_only_function)

template<typename Sig>
using move_only_function = std::move_only_function<Sig>;

#else

inline constexpr std::size_t callable_buffer_bytes = 64;

template<typename Sig>
class move_only_function;

template<typename R, typename... Args>
class move_only_function<R(Args...)>
{
    // A hand-written dispatch table rather than a virtual base, so a target that fits the buffer
    // needs no allocation at all. relocate leaves the source holding no target, and a null table is
    // the empty state.
    struct operations
    {
        R (*invoke)(std::byte *buffer, Args... args);
        void (*relocate)(std::byte *to, std::byte *from) noexcept;
        void (*destroy)(std::byte *buffer) noexcept;
    };

public:
    move_only_function()
            : m_ops(nullptr)
    {
    }

    move_only_function(std::nullptr_t) noexcept
            : m_ops(nullptr)
    {
    }

    template<typename F, std::enable_if_t<!std::is_same_v<std::remove_cvref_t<F>, move_only_function> && std::is_invocable_r_v<R, F &, Args...>, int> = 0>
    move_only_function(F target)
            : m_ops(nullptr)
    {
        emplace<std::decay_t<F>>(std::move(target));
    }

    move_only_function(const move_only_function &)            = delete;
    move_only_function &operator=(const move_only_function &) = delete;

    move_only_function(move_only_function &&other) noexcept
            : m_ops(nullptr)
    {
        steal(other);
    }

    move_only_function &operator=(move_only_function &&other) noexcept
    {
        if(this != &other)
        {
            reset();
            steal(other);
        }

        return *this;
    }

    ~move_only_function()
    {
        reset();
    }

    explicit operator bool() const noexcept
    {
        return m_ops != nullptr;
    }

    friend bool operator==(const move_only_function &target, std::nullptr_t) noexcept
    {
        return target.m_ops == nullptr;
    }

    R operator()(Args... args)
    {
        if(m_ops == nullptr)
            called_without_target();

        return m_ops->invoke(m_buffer, std::forward<Args>(args)...);
    }

private:
    const operations *m_ops;
    alignas(std::max_align_t) std::byte m_buffer[callable_buffer_bytes];

    // Inline when the target fits the budget, is not over-aligned and moves without throwing;
    // otherwise the buffer holds one owning pointer to it.
    template<typename F>
    static constexpr bool fits_inline = sizeof(F) <= callable_buffer_bytes && alignof(F) <= alignof(std::max_align_t) && std::is_nothrow_move_constructible_v<F>;

    template<typename F>
    static F *as(std::byte *buffer) noexcept
    {
        if constexpr(fits_inline<F>)
            return reinterpret_cast<F *>(buffer);
        else
            return *reinterpret_cast<F **>(buffer);
    }

    template<typename F>
    static const operations *table_for() noexcept
    {
        static constexpr operations table{[](std::byte *buffer, Args... args) -> R { return (*as<F>(buffer))(std::forward<Args>(args)...); },
                                          [](std::byte *to, std::byte *from) noexcept
                                          {
                                              if constexpr(fits_inline<F>)
                                              {
                                                  ::new(to) F(std::move(*as<F>(from)));
                                                  as<F>(from)->~F();
                                              }
                                              else
                                                  *reinterpret_cast<F **>(to) = *reinterpret_cast<F **>(from);
                                          },
                                          [](std::byte *buffer) noexcept
                                          {
                                              if constexpr(fits_inline<F>)
                                                  as<F>(buffer)->~F();
                                              else
                                                  delete as<F>(buffer);
                                          }};
        return &table;
    }

    void reset() noexcept
    {
        if(m_ops != nullptr)
            m_ops->destroy(m_buffer);
        m_ops = nullptr;
    }

    template<typename F>
    void emplace(F target)
    {
        if constexpr(fits_inline<F>)
            ::new(static_cast<void *>(m_buffer)) F(std::move(target));
        else
            *reinterpret_cast<F **>(m_buffer) = new F(std::move(target));
        m_ops = table_for<F>();
    }

    void steal(move_only_function &other) noexcept
    {
        if(other.m_ops != nullptr)
            other.m_ops->relocate(m_buffer, other.m_buffer);
        m_ops       = other.m_ops;
        other.m_ops = nullptr;
    }
};

#endif

}

#endif
