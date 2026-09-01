#include "key_path.h"

#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <string_view>

namespace praxis::config {

std::string declared_path(std::string_view key)
{
    std::string plain;
    plain.reserve(key.size());
    bool inside = false;
    for(const char letter : key)
    {
        if(letter == '[')
            inside = true;
        else if(letter == ']')
            inside = false;
        else if(!inside)
            plain.push_back(letter);
    }
    return plain;
}

std::vector<std::string_view> segments_of(std::string_view path)
{
    std::vector<std::string_view> parts;
    std::size_t from = 0;
    while(from <= path.size())
    {
        const std::size_t cut = path.find('/', from);
        if(cut == std::string_view::npos)
        {
            parts.push_back(path.substr(from));
            break;
        }
        parts.push_back(path.substr(from, cut - from));
        from = cut + 1;
    }
    return parts;
}

std::size_t segments_in(std::string_view path)
{
    return static_cast<std::size_t>(std::count(path.begin(), path.end(), '/')) + 1;
}

std::string leading_segments(const std::string &path, std::size_t count)
{
    std::size_t cut  = std::string::npos;
    std::size_t from = 0;
    for(std::size_t taken = 0; taken < count; ++taken)
    {
        cut = path.find('/', from);
        if(cut == std::string::npos)
            return path;
        from = cut + 1;
    }
    return path.substr(0, cut);
}

bool hangs_under(std::string_view path, std::string_view ancestor)
{
    return path.size() > ancestor.size() + 1 && path.compare(0, ancestor.size(), ancestor) == 0 && path[ancestor.size()] == '/';
}

}
