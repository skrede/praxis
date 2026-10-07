#include "engine.h"

#include <map>
#include <set>
#include <string>
#include <cstddef>
#include <optional>
#include <functional>

namespace praxis::config {
namespace {

struct declared_so_far
{
    std::map<std::string, node_kind, std::less<>> shapes;
    std::set<std::string, std::less<>> identities;
};

std::string parent_of(const std::string &path)
{
    const std::size_t cut = path.rfind('/');
    return cut == std::string::npos ? std::string() : path.substr(0, cut);
}

bool holds_nodes(const declared_so_far &seen, const std::string &path)
{
    const auto found = seen.shapes.find(path);
    return found != seen.shapes.end() && found->second != node_kind::leaf;
}

std::optional<std::string> finding(const node &declared, const declared_so_far &seen)
{
    const std::string parent = parent_of(declared.path);
    if(seen.shapes.contains(declared.path))
        return "'" + declared.path + "' is declared twice";
    if(seen.identities.contains(declared.path))
        return "'" + declared.path + "' is the identity of the collection '" + parent + "' and is declared again";
    if(!parent.empty() && !holds_nodes(seen, parent))
        return "'" + declared.path + "' hangs under '" + parent + "', which is declared before it as neither a group nor a collection";
    return std::nullopt;
}

}

std::optional<error> refused_as_declared(const declaration &shape)
{
    declared_so_far seen;
    std::string findings = shape.space().empty() ? "the space is empty, so no document can be rooted at it" : "";
    for(const node &declared : shape.nodes())
    {
        if(const std::optional<std::string> found = finding(declared, seen); found)
            findings += (findings.empty() ? "" : "; ") + *found;
        seen.shapes.emplace(declared.path, declared.shape);
        if(declared.shape == node_kind::collection)
            seen.identities.insert(declared.path + "/" + declared.identity);
    }
    if(findings.empty())
        return std::nullopt;
    return error{error_code::malformed_source, "the declaration of the space '" + shape.space() + "' cannot describe a keyspace: " + findings};
}

}
