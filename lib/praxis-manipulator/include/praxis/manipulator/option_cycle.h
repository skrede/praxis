#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_OPTION_CYCLE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_OPTION_CYCLE_H

#include <span>
#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

namespace detail {

std::optional<std::uint8_t> index_of_label(std::span<const char *const> labels, std::string_view label);

}

// A fixed set of options paired with the labels a combo box shows, holding the one that is
// selected. A label the set does not carry is not an error: the lookup reports absence and the
// caller decides what an unknown label means.
template<typename E, std::uint8_t N>
class option_cycle
{
public:
    option_cycle(E initial, std::array<E, N> options, std::array<const char *, N> labels)
            : m_value(initial)
            , m_values(options)
            , m_labels(labels)
    {
    }

    E value() const
    {
        return m_value;
    }

    void set(E option)
    {
        m_value = option;
    }

    bool set(std::string_view label)
    {
        const auto option = option_of(label);
        if(option)
            m_value = *option;

        return option.has_value();
    }

    std::optional<E> option_of(std::string_view label) const
    {
        const auto index = detail::index_of_label(m_labels, label);
        if(!index)
            return std::nullopt;

        return m_values[*index];
    }

    std::optional<std::uint8_t> value_index() const
    {
        return index_of(m_value);
    }

    std::string_view label() const
    {
        return label(m_value);
    }

    std::string_view label(E option) const
    {
        const auto index = index_of(option);

        return index ? std::string_view(m_labels[*index]) : std::string_view();
    }

    // Both spans alias this object's storage and are valid only as long as it is.
    std::span<const E> options() const
    {
        return m_values;
    }

    std::span<const char *const> labels() const
    {
        return m_labels;
    }

    bool operator==(E rhs) const
    {
        return m_value == rhs;
    }

    bool operator==(std::string_view rhs) const
    {
        return option_of(rhs) == std::optional<E>(m_value);
    }

private:
    E m_value;
    std::array<E, N> m_values;
    std::array<const char *, N> m_labels;

    std::optional<std::uint8_t> index_of(E option) const
    {
        for(std::uint8_t i = 0; i < N; ++i)
            if(m_values[i] == option)
                return i;

        return std::nullopt;
    }
};

}

#endif
