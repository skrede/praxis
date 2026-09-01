#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_HELD_HOME_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_HELD_HOME_H

#include <array>
#include <string>
#include <cstddef>
#include <cstdlib>
#include <optional>

namespace praxis::tests {

// Every variable the resolver reads for a home directory, in the order it reads them. A fixture
// that governs fewer than all of them cannot leave the resolver with no home directory to read.
constexpr auto home_variables()
{
#ifdef _WIN32
    return std::array<const char *, 3>{"USERPROFILE", "HOMEDRIVE", "HOMEPATH"};
#else
    return std::array<const char *, 1>{"HOME"};
#endif
}

inline std::optional<std::string> variable_read(const char *name)
{
    const char *set = std::getenv(name);
    return set == nullptr ? std::optional<std::string>() : std::optional<std::string>(set);
}

// Setting a variable to an empty value removes it on Windows, so a variable carrying nothing and a
// variable that is not there are one state on that platform and two on the others.
inline void variable_written(const char *name, const std::optional<std::string> &value)
{
#ifdef _WIN32
    _putenv_s(name, value ? value->c_str() : "");
#else
    if(value)
        setenv(name, value->c_str(), 1);
    else
        unsetenv(name);
#endif
}

// The environment a case dictates is put back as it was even where an assertion throws out of the
// case, so a run's later cases read the machine's own home directory rather than a scratch one. A
// value that is not there leaves the variables unset; an empty one leaves them set and carrying
// nothing.
class held_home
{
public:
    explicit held_home(const std::optional<std::string> &value)
    {
        constexpr auto named = home_variables();
        for(std::size_t at = 0u; at < named.size(); ++at)
            m_before[at] = variable_read(named[at]);
        // The first variable carries the value and the rest are unset, so a home directory is
        // readable exactly where a case asked for one.
        for(std::size_t at = 0u; at < named.size(); ++at)
            variable_written(named[at], at == 0u ? value : std::optional<std::string>());
    }

    held_home(const held_home &)            = delete;
    held_home &operator=(const held_home &) = delete;

    ~held_home()
    {
        constexpr auto named = home_variables();
        for(std::size_t at = 0u; at < named.size(); ++at)
            variable_written(named[at], m_before[at]);
    }

private:
    std::array<std::optional<std::string>, home_variables().size()> m_before;
};

}

#endif
