#ifndef HPP_GUARD_PRAXIS_EXTENSION_SLOT_SET_H
#define HPP_GUARD_PRAXIS_EXTENSION_SLOT_SET_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace praxis {

template<typename Slot>
concept slot_enumeration = std::is_enum_v<Slot> && requires { Slot::count; };

template<slot_enumeration Slot>
class basic_slot_set
{
    static constexpr std::size_t word_bits     = 64;
    static constexpr std::size_t slot_count    = static_cast<std::size_t>(Slot::count);
    static constexpr std::size_t word_count    = (slot_count + word_bits - 1) / word_bits;
    static constexpr std::size_t trailing_bits = slot_count % word_bits;

    static constexpr std::uint64_t last_word_mask = trailing_bits == 0 ? ~std::uint64_t{0} : (std::uint64_t{1} << trailing_bits) - 1;

    static_assert(slot_count > 0, "a capability declares at least one slot before its count enumerator");

public:
    constexpr basic_slot_set()
            : m_words{}
    {
    }

    constexpr basic_slot_set &set(Slot s)
    {
        if(is_a_slot(s))
        {
            m_words[word_of(s)] |= bit_of(s);
        }
        return *this;
    }

    constexpr bool contains(Slot s) const
    {
        return is_a_slot(s) && (m_words[word_of(s)] & bit_of(s)) != 0;
    }

    constexpr bool empty() const
    {
        for(std::uint64_t word : m_words)
        {
            if(word != 0)
            {
                return false;
            }
        }
        return true;
    }

    constexpr basic_slot_set operator|(const basic_slot_set &other) const
    {
        basic_slot_set result;
        for(std::size_t i = 0; i < word_count; ++i)
        {
            result.m_words[i] = m_words[i] | other.m_words[i];
        }
        return result;
    }

    constexpr basic_slot_set operator&(const basic_slot_set &other) const
    {
        basic_slot_set result;
        for(std::size_t i = 0; i < word_count; ++i)
        {
            result.m_words[i] = m_words[i] & other.m_words[i];
        }
        return result;
    }

    // The high bits of the last word address no slot, so the complement clears them; otherwise the
    // complement of the set of every slot would report itself non-empty.
    constexpr basic_slot_set operator~() const
    {
        basic_slot_set result;
        for(std::size_t i = 0; i < word_count; ++i)
        {
            result.m_words[i] = ~m_words[i];
        }
        result.m_words[word_count - 1] &= last_word_mask;
        return result;
    }

private:
    std::array<std::uint64_t, word_count> m_words;

    static constexpr bool is_a_slot(Slot s)
    {
        return static_cast<std::size_t>(s) < slot_count;
    }

    static constexpr std::size_t word_of(Slot s)
    {
        return static_cast<std::size_t>(s) / word_bits;
    }

    static constexpr std::uint64_t bit_of(Slot s)
    {
        return std::uint64_t{1} << (static_cast<std::size_t>(s) % word_bits);
    }
};

}

#endif
