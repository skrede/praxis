#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_LIST_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_LIST_H

#include "praxis/optimization/problem.h"

#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

template<typename registered>
std::vector<std::string> names_of(const std::vector<registered> &functions)
{
    std::vector<std::string> named;
    for(const registered &function : functions)
        named.push_back(function.name);

    return named;
}

template<typename registered, typename posing>
std::vector<problem> posed_all(const std::vector<registered> &functions, posing pose, double tolerance)
{
    std::vector<problem> posed;
    for(const registered &function : functions)
        posed.push_back(pose(function, tolerance));

    return posed;
}

// The functions a scenario in that form lists and indexes, in the order handed.
template<typename registered>
std::vector<registered> offered_in(std::vector<registered> functions, problem_form form)
{
    std::erase_if(functions, [form](const registered &function) { return std::ranges::find(function.forms, form) == function.forms.end(); });

    return functions;
}

inline std::optional<std::size_t> index_of(const std::vector<std::string> &names, const std::string &name)
{
    const auto found = std::find(names.begin(), names.end(), name);
    if(found == names.end())
        return std::nullopt;

    return static_cast<std::size_t>(found - names.begin());
}

}

#endif
