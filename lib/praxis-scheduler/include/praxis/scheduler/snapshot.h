#ifndef HPP_GUARD_PRAXIS_SCHEDULER_SNAPSHOT_H
#define HPP_GUARD_PRAXIS_SCHEDULER_SNAPSHOT_H

#include <mutex>
#include <memory>
#include <utility>

namespace praxis::scheduler {

template<typename value_type>
class snapshot_publisher;

// A value one strand hands to another. The publisher and every reader taken from it hold the same
// cell, so neither side's destruction leaves the other reading storage that is gone, and a reader
// stays a cheap copyable value whose read costs one copy of the published type.
template<typename value_type>
class snapshot_reader
{
    class cell
    {
    public:
        cell()
                : m_value()
                , m_mutex()
        {
        }

        value_type load() const
        {
            const std::lock_guard held(m_mutex);

            return m_value;
        }

        void store(value_type published)
        {
            const std::lock_guard held(m_mutex);
            m_value = std::move(published);
        }

    private:
        value_type m_value;
        mutable std::mutex m_mutex;
    };

public:
    value_type read() const
    {
        return m_held->load();
    }

private:
    friend class snapshot_publisher<value_type>;

    std::shared_ptr<const cell> m_held;

    explicit snapshot_reader(std::shared_ptr<const cell> held)
            : m_held(std::move(held))
    {
    }
};

// Publication and reading are the whole surface, so what holds the value between them can be
// replaced without a caller moving.
template<typename value_type>
class snapshot_publisher
{
public:
    snapshot_publisher()
            : m_held(std::make_shared<typename snapshot_reader<value_type>::cell>())
    {
    }

    void publish(value_type value)
    {
        m_held->store(std::move(value));
    }

    snapshot_reader<value_type> reader() const
    {
        return snapshot_reader<value_type>(m_held);
    }

private:
    std::shared_ptr<typename snapshot_reader<value_type>::cell> m_held;
};

}

#endif
