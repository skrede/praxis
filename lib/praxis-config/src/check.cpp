#include "check.h"
#include "engine.h"
#include "nearest.h"
#include "key_path.h"

#include <map>
#include <set>
#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <string_view>

namespace praxis::config {
namespace {

struct findings
{
    std::vector<std::string> structural;
    std::vector<std::string> content;
};

bool reads_as_its_kind(field_kind kind, std::string_view text)
{
    switch(kind)
    {
        case field_kind::flag:
            return as_flag(text).has_value();
        case field_kind::integer:
            return as_integer(text).has_value();
        case field_kind::real:
            return as_real(text).has_value();
        case field_kind::text:
        case field_kind::choice:
            return true;
    }
    return true;
}

std::string named_kind(field_kind kind)
{
    switch(kind)
    {
        case field_kind::flag:
            return "a flag";
        case field_kind::integer:
            return "an integer";
        case field_kind::real:
            return "a real number";
        case field_kind::text:
        case field_kind::choice:
            return "text";
    }
    return "text";
}

std::string listed(const std::vector<std::string> &allowed)
{
    std::string named;
    for(const std::string &one : allowed)
        named += (named.empty() ? "'" : ", '") + one + "'";
    return named;
}

// Every declared path, a collection's identity included.
std::vector<std::string> paths_in(const declaration &shape)
{
    std::vector<std::string> paths;
    for(const node &declared : shape.nodes())
    {
        paths.push_back(declared.path);
        if(declared.shape == node_kind::collection)
            paths.push_back(declared.path + "/" + declared.identity);
    }
    return paths;
}

std::string nearest_of(std::string_view to, std::span<const std::string> among, std::string_view called)
{
    const std::string near = nearest(to, among);
    return near.empty() ? std::string() : "; the nearest " + std::string(called) + " is '" + near + "'";
}

// A choice is compared byte for byte, and one declared with no values to choose from reads as text.
std::optional<std::string> value_fault(const node &leaf, const std::string &key, const std::string &text)
{
    const std::string carried = "'" + key + "' is '" + text + "', which ";
    if(leaf.kind == field_kind::choice && !leaf.allowed.empty() && std::ranges::find(leaf.allowed, text) == leaf.allowed.end())
        return carried + "is none of " + listed(leaf.allowed) + nearest_of(text, leaf.allowed, "allowed value");
    if(!reads_as_its_kind(leaf.kind, text))
        return carried + "does not read as " + named_kind(leaf.kind);
    return std::nullopt;
}

void check_values(const detail::entry_map &entries, const declaration &shape, findings &found)
{
    std::map<std::string, const node *, std::less<>> leaves;
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::leaf)
            leaves.emplace(declared.path, &declared);

    for(const std::pair<const std::string, std::string> &value : entries)
        if(const auto leaf = leaves.find(declared_path(value.first)); leaf != leaves.end())
            if(const std::optional<std::string> fault = value_fault(*leaf->second, value.first, value.second); fault)
                found.content.push_back(*fault);
}

std::set<std::string> instances_of(const detail::entry_map &entries, const std::string &collection_path)
{
    const std::size_t depth = segments_in(collection_path);
    std::set<std::string> instances;
    for(const std::pair<const std::string, std::string> &value : entries)
        if(segments_in(value.first) > depth)
            if(std::string instance = leading_segments(value.first, depth); declared_path(instance) == collection_path)
                instances.insert(std::move(instance));
    return instances;
}

// An identity is required on every instance that carries a value, and distinct among the instances
// one holder carries; a repeat among the instances no collection encloses is structural.
void check_identities(const detail::entry_map &entries, const node &collection, findings &found)
{
    std::set<std::pair<std::string, std::string>> claimed;
    for(const std::string &instance : instances_of(entries, collection.path))
    {
        const detail::entry_map::const_iterator identity = entries.find(instance + "/" + collection.identity);
        if(identity == entries.end())
        {
            found.content.push_back("the instance '" + instance + "' carries no '" + collection.identity + "'");
            continue;
        }

        const std::string holder = instance.substr(0, instance.rfind('['));
        if(claimed.emplace(holder, identity->second).second)
            continue;
        std::vector<std::string> &kept = instance.find('[') == instance.rfind('[') ? found.structural : found.content;
        kept.push_back("'" + identity->first + "' is '" + identity->second + "', which another instance of '" + holder + "' already carries");
    }
}

}

std::optional<error> refused_content(const folding &walked, const declaration &shape, const std::filesystem::path &from)
{
    findings found{walked.malformed, {}};
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::collection)
            check_identities(walked.entries, declared, found);
    const std::vector<std::string> declared = paths_in(shape);
    for(const std::string &key : walked.undeclared)
        found.content.push_back("'" + key + "' is not declared" + nearest_of(declared_path(key), declared, "declared path"));
    check_values(walked.entries, shape, found);
    if(found.structural.empty() && found.content.empty())
        return std::nullopt;

    std::string message = "the configuration at " + from.string() + " cannot be used:";
    for(const std::vector<std::string> *kept : {&found.structural, &found.content})
        for(const std::string &one : *kept)
            message += "\n  - " + one;
    return error{found.structural.empty() ? error_code::rejected_content : error_code::malformed_source, std::move(message)};
}

}
