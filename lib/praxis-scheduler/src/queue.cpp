#include "queue.h"

#include <vector>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::scheduler {

namespace {

constexpr std::uint32_t ready_burst = 1;

// std::push_heap puts the greatest element in front, so ordering by "runs later" puts the nearest
// deadline there.
bool runs_later(const armed_task &first, const armed_task &second)
{
    if(first.due() != second.due())
        return first.due() > second.due();
    return first.id > second.id;
}

}

time_point armed_task::due() const
{
    return std::visit([](const auto &task) { return task.due(); }, work);
}

task_counters armed_task::counters() const
{
    return std::visit([](const auto &task) { return task.counters(); }, work);
}

void armed_task::arm(time_point now)
{
    std::visit([now](auto &task) { task.arm(now); }, work);
}

void armed_task::run(time_point now)
{
    std::visit([now](auto &task) { task.run(now); }, work);
}

strand_queue::strand_queue()
        : m_retired(false)
        , m_occupied(false)
        , m_head(0)
        , m_earliest(time_point::max())
        , m_sample_due(time_point::max())
        , m_sample_time()
        , m_ready_streak(0)
        , m_armed()
        , m_ready()
{
}

bool strand_queue::select_ready(time_point now)
{
    if(!ready())
        return false;
    if(m_earliest > now)
    {
        m_ready_streak = 0;
        return true;
    }
    if(m_ready_streak >= ready_burst)
        return false;
    ++m_ready_streak;

    return true;
}

time_point strand_queue::earliest() const
{
    return m_earliest;
}

time_point strand_queue::delivery_time(const armed_task &entry, time_point now)
{
    if(!entry.sampled())
        return now;
    if(entry.due() != m_sample_due)
    {
        m_sample_due  = entry.due();
        m_sample_time = now;
    }

    return m_sample_time;
}

// A retired strand has no future, so its deadlines go with its admissions; what was already posted
// stays, because the queue is what the strand still has to finish.
std::vector<armed_task> strand_queue::retire()
{
    m_retired = true;
    std::vector<armed_task> discarded;
    discarded.swap(m_armed);
    reseat();

    return discarded;
}

discarded_work strand_queue::evacuate()
{
    discarded_work discarded{m_head, {}, {}};
    discarded.armed.swap(m_armed);
    discarded.ready.swap(m_ready);
    m_head     = 0;
    m_earliest = time_point::max();

    return discarded;
}

void strand_queue::occupy()
{
    m_occupied = true;
}

void strand_queue::release()
{
    m_occupied = false;
}

void strand_queue::push(detail::move_only_function<void()> &&work)
{
    m_ready.push_back(std::move(work));
}

// A vector cleared once its head reaches the end keeps the capacity a steady post-and-advance loop
// would otherwise pay a deque's block-map recentering for.
detail::move_only_function<void()> strand_queue::take()
{
    detail::move_only_function<void()> work = std::move(m_ready[m_head]);
    if(++m_head == m_ready.size())
    {
        m_ready.clear();
        m_head = 0;
    }

    return work;
}

void strand_queue::arm(armed_task &&entry)
{
    m_armed.push_back(std::move(entry));
    std::push_heap(m_armed.begin(), m_armed.end(), runs_later);
    reseat();
}

armed_task strand_queue::take_due()
{
    std::pop_heap(m_armed.begin(), m_armed.end(), runs_later);
    armed_task entry = std::move(m_armed.back());
    m_armed.pop_back();
    m_ready_streak = 0;
    reseat();

    return entry;
}

bool strand_queue::contains(std::uint64_t id) const
{
    return std::any_of(m_armed.begin(), m_armed.end(), [id](const armed_task &entry) { return entry.id == id; });
}

std::optional<task_counters> strand_queue::counters(std::uint64_t id) const
{
    const auto found = std::find_if(m_armed.begin(), m_armed.end(), [id](const armed_task &entry) { return entry.id == id; });
    if(found == m_armed.end())
        return std::nullopt;

    return found->counters();
}

std::optional<armed_task> strand_queue::drop(std::uint64_t id, time_point now)
{
    const auto found = std::find_if(m_armed.begin(), m_armed.end(), [id](const armed_task &entry) { return entry.id == id; });
    if(found == m_armed.end())
        return std::nullopt;

    armed_task discarded = std::move(*found);
    if(found != m_armed.end() - 1)
        *found = std::move(m_armed.back());
    m_armed.pop_back();
    std::make_heap(m_armed.begin(), m_armed.end(), runs_later);
    reseat();
    if(m_earliest > now)
        m_ready_streak = 0;

    return discarded;
}

void strand_queue::reseat()
{
    m_earliest = m_armed.empty() ? time_point::max() : m_armed.front().due();
}

}
