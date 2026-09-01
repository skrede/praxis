#ifndef HPP_GUARD_PRAXIS_EXTENSION_DESCRIPTOR_H
#define HPP_GUARD_PRAXIS_EXTENSION_DESCRIPTOR_H

#include <span>
#include <string_view>

namespace praxis {

// Declaration order in both aggregates below is frozen: every descriptor table is written as a brace
// initializer, so reordering a member silently changes what an existing table means. Appending is
// safe.
struct slot_descriptor
{
    std::string_view name;
    bool (*holds_default)(const void *value);
};

template<typename Ops>
struct capability_descriptors
{
    std::string_view extension;
    std::span<const slot_descriptor> slots;
};

class capability_view
{
public:
    capability_view()
            : m_value(nullptr)
            , m_extension()
            , m_slots()
    {
    }

    // Ops is deduced from both arguments, so a value described by another capability's table is a
    // deduction conflict rather than a predicate casting the pointer to a type nobody checked.
    template<typename Ops>
    static capability_view of(const Ops &ops, const capability_descriptors<Ops> &described)
    {
        return capability_view(&ops, described.extension, described.slots);
    }

    template<typename Ops>
    static capability_view of(Ops &&, const capability_descriptors<Ops> &) = delete;

    const void *value() const
    {
        return m_value;
    }

    std::string_view extension() const
    {
        return m_extension;
    }

    std::span<const slot_descriptor> slots() const
    {
        return m_slots;
    }

private:
    const void *m_value;
    std::string_view m_extension;
    std::span<const slot_descriptor> m_slots;

    capability_view(const void *value, std::string_view extension, std::span<const slot_descriptor> slots)
            : m_value(value)
            , m_extension(extension)
            , m_slots(slots)
    {
    }
};

}

#endif
