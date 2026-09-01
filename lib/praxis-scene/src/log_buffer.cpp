#include "praxis/scene/log_buffer.h"

#include <spdlog/spdlog.h>

#include <array>
#include <mutex>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::scene {

namespace {

// The one correspondence between the published vocabulary and the logger's levels. Read forwards it
// is the threshold a chosen level asks for; read backwards it answers which published level a
// logger holding an unpublished one behaves as.
spdlog::level::level_enum logger_level(severity level)
{
    switch(level)
    {
        case severity::debug:
            return spdlog::level::debug;
        case severity::info:
            return spdlog::level::info;
        case severity::warning:
            return spdlog::level::warn;
        case severity::error:
            return spdlog::level::err;
    }

    return spdlog::level::info;
}

constexpr std::array<severity, 4> published_levels{severity::debug, severity::info, severity::warning, severity::error};

}

bool same_message(const log_entry &first, const log_entry &second)
{
    return first.level == second.level && first.text == second.text;
}

log_buffer::log_buffer(std::size_t capacity)
        : m_size(0)
        , m_first(0)
        , m_dropped(0)
        , m_mutex()
        , m_entries(std::max<std::size_t>(capacity, 1))
{
}

void log_buffer::push(std::string text, severity level)
{
    log_entry incoming{std::move(text), level, 1};
    const std::lock_guard held(m_mutex);

    if(m_size > 0 && same_message(m_entries[newest()], incoming))
        ++m_entries[newest()].repeats;
    else
        append(std::move(incoming));
}

std::vector<log_entry> log_buffer::drain()
{
    std::vector<log_entry> drained;
    const std::lock_guard held(m_mutex);
    take(drained);

    return drained;
}

std::size_t log_buffer::dropped() const
{
    const std::lock_guard held(m_mutex);

    return m_dropped;
}

std::size_t log_buffer::capacity() const
{
    const std::lock_guard held(m_mutex);

    return m_entries.size();
}

std::size_t log_buffer::newest() const
{
    return (m_first + m_size - 1) % m_entries.size();
}

void log_buffer::append(log_entry entry)
{
    m_entries[(m_first + m_size) % m_entries.size()] = std::move(entry);
    if(m_size < m_entries.size())
    {
        ++m_size;
        return;
    }
    m_first = (m_first + 1) % m_entries.size();
    ++m_dropped;
}

void log_buffer::take(std::vector<log_entry> &out)
{
    out.reserve(m_size);
    for(std::size_t i = 0; i < m_size; ++i)
        out.push_back(std::move(m_entries[(m_first + i) % m_entries.size()]));
    m_size  = 0;
    m_first = 0;
}

void set_reporting_level(severity level)
{
    spdlog::default_logger()->set_level(logger_level(level));
}

severity reporting_level()
{
    const spdlog::level::level_enum admitted = spdlog::default_logger()->level();
    for(severity candidate : published_levels)
        if(logger_level(candidate) >= admitted)
            return candidate;

    return severity::error;
}

}
