#include "nearest.h"

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <string_view>

namespace praxis::config {
namespace {

constexpr std::size_t half_edit = 1;
constexpr std::size_t edit      = 2;

int class_of(char c)
{
    if(c >= 'a' && c <= 'z')
        return 1;
    return c >= '0' && c <= '9' ? 2 : 0;
}

std::size_t substitution(char from, char to)
{
    if(from == to)
        return 0;
    return class_of(from) != 0 && class_of(from) == class_of(to) ? half_edit : edit;
}

// Weighted Levenshtein distance.
std::size_t distance(std::string_view from, std::string_view to)
{
    std::vector<std::size_t> above(to.size() + 1);
    std::vector<std::size_t> row(to.size() + 1);
    for(std::size_t j = 0; j <= to.size(); ++j)
        above[j] = j * edit;
    for(std::size_t i = 1; i <= from.size(); ++i)
    {
        row[0] = i * edit;
        for(std::size_t j = 1; j <= to.size(); ++j)
            row[j] = std::min({above[j] + edit, row[j - 1] + edit, above[j - 1] + substitution(from[i - 1], to[j - 1])});
        std::swap(above, row);
    }
    return above[to.size()];
}

}

std::string nearest(std::string_view to, std::span<const std::string> among)
{
    const std::string *best = nullptr;
    std::size_t least       = 0;
    for(const std::string &candidate : among)
    {
        const std::size_t cost = distance(to, candidate);
        if(best == nullptr || cost < least || (cost == least && candidate < *best))
        {
            best  = &candidate;
            least = cost;
        }
    }
    return best == nullptr ? std::string() : *best;
}

}
