#ifndef HPP_GUARD_PRAXIS_SCENE_LOG_BUFFER_H
#define HPP_GUARD_PRAXIS_SCENE_LOG_BUFFER_H

#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>

namespace praxis::scene {

enum class severity : std::uint8_t
{
    debug,
    info,
    warning,
    error
};

struct log_entry
{
    std::string text;
    severity level;
    std::uint32_t repeats;
};

inline constexpr std::size_t default_log_capacity = 512;

bool same_message(const log_entry &first, const log_entry &second);

class log_buffer
{
public:
    explicit log_buffer(std::size_t capacity);

    void push(std::string text, severity level);

    std::vector<log_entry> drain();

    std::size_t dropped() const;

    std::size_t capacity() const;

private:
    std::size_t m_size;
    std::size_t m_first;
    std::size_t m_dropped;
    mutable std::mutex m_mutex;
    std::vector<log_entry> m_entries;

    // The three below are called with m_mutex already held.
    std::size_t newest() const;

    void append(log_entry entry);

    void take(std::vector<log_entry> &out);
};

std::size_t install_log_sink(std::shared_ptr<log_buffer> messages);

// The level belongs to the logger every sink hangs off, and the logger consults it once before any
// sink is invoked, so one setting governs the terminal and the message window together.
void set_reporting_level(severity level);

// The lowest published level the logger still admits, which is what was last set whenever the level
// held is one of the four.
severity reporting_level();

}

#endif
