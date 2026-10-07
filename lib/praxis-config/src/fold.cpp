#include "fold.h"
#include "key_path.h"

#include <pugixml.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <functional>
#include <string_view>

namespace praxis::config {
namespace {

constexpr std::size_t deepest_element = 64;

// `values` holds every path a value may stand at, which leaves out the collections themselves;
// `leaves` those of them no group stands at: the declared leaves and every collection's identity.
struct walk
{
    folding &out;
    std::set<std::string, std::less<>> collections;
    std::set<std::string, std::less<>> values;
    std::set<std::string, std::less<>> leaves;
    std::map<std::string, std::size_t, std::less<>> ordinals;
};

walk walking(const declaration &shape, folding &out)
{
    walk state{out, {}, {}, {}, {}};
    for(const node &declared : shape.nodes())
    {
        if(declared.shape == node_kind::collection)
            state.collections.insert(declared.path);
        std::string value = declared.shape == node_kind::collection ? declared.path + "/" + declared.identity : declared.path;
        if(declared.shape != node_kind::group)
            state.leaves.insert(value);
        state.values.insert(std::move(value));
    }
    return state;
}

bool blank(std::string_view text)
{
    return text.find_first_not_of(" \t\r\n") == std::string_view::npos;
}

// Whitespace between elements is no value, but a CDATA section is one whatever it holds.
bool carries_text(pugi::xml_node element)
{
    return !element.find_child([](pugi::xml_node piece) { return piece.type() == pugi::node_cdata || (piece.type() == pugi::node_pcdata && !blank(piece.value())); }).empty();
}

bool carries_elements(pugi::xml_node element)
{
    return !element.find_child([](pugi::xml_node piece) { return piece.type() == pugi::node_element; }).empty();
}

std::string text_of(pugi::xml_node element)
{
    std::string joined;
    for(const pugi::xml_node piece : element.children())
        if(piece.type() == pugi::node_pcdata || piece.type() == pugi::node_cdata)
            joined += piece.value();
    return joined;
}

std::string joined(const std::string &path, std::string_view name)
{
    return path.empty() ? std::string(name) : path + "/" + std::string(name);
}

// A leaf or identity written twice is a fault; a group written again takes the later copy.
void kept(walk &state, std::string key, std::string text)
{
    const std::string declared = declared_path(key);
    if(!state.values.contains(declared))
        state.out.undeclared.push_back(std::move(key));
    else if(!state.leaves.contains(declared))
        state.out.entries.insert_or_assign(std::move(key), std::move(text));
    else if(!state.out.entries.try_emplace(key, std::move(text)).second)
        if(std::string finding = "'" + key + "' is written twice"; std::ranges::find(state.out.malformed, finding) == state.out.malformed.end())
            state.out.malformed.push_back(std::move(finding));
}

std::optional<std::string> structural_fault(pugi::xml_node element, const std::string &where)
{
    if(!carries_text(element))
        return std::nullopt;
    const char *beside = carries_elements(element) ? "beside child elements" : element.first_attribute() ? "beside attributes" : "where only elements and attributes can stand";
    return "'" + where + "' carries the text '" + text_of(element) + "' " + beside;
}

// The attributes of the root element of a named space are not values.
void walk_attributes(walk &state, pugi::xml_node element, const std::string &path)
{
    if(path.empty())
        return;
    std::set<std::string_view> seen;
    for(const pugi::xml_attribute carried : element.attributes())
    {
        std::string key = joined(path, carried.name());
        if(seen.insert(carried.name()).second)
            kept(state, std::move(key), carried.value());
        else if(state.values.contains(declared_path(key)))
            state.out.malformed.push_back("'" + key + "' is written twice on one element");
    }
}

void walk_undeclared(walk &state, pugi::xml_node element, const std::string &path, std::size_t depth);

// The attributes and child elements of an element at `path`, none of them at a declared path.
void undeclared_within(walk &state, pugi::xml_node element, const std::string &path, std::size_t depth)
{
    for(const pugi::xml_attribute carried : element.attributes())
        state.out.undeclared.push_back(joined(path, carried.name()));
    for(const pugi::xml_node child : element.children())
        if(child.type() == pugi::node_element)
            walk_undeclared(state, child, joined(path, child.name()), depth + 1);
}

// Content at a path nothing declares is named and never checked, and not descended into past the depth bound.
void walk_undeclared(walk &state, pugi::xml_node element, const std::string &path, std::size_t depth)
{
    if(depth > deepest_element || carries_text(element) || (!element.first_attribute() && !carries_elements(element)))
        state.out.undeclared.push_back(path);
    if(depth <= deepest_element)
        undeclared_within(state, element, path, depth);
}

void walk_element(walk &state, pugi::xml_node element, const std::string &path, std::size_t depth);

void walk_child(walk &state, pugi::xml_node child, const std::string &path, std::size_t depth)
{
    const std::string declared = declared_path(path);
    if(state.collections.contains(declared))
    {
        const std::string instance = path + "[" + std::to_string(state.ordinals[path]++) + "]";
        state.out.instances.push_back(instance);
        walk_element(state, child, instance, depth);
    }
    else if(state.leaves.contains(declared))
    {
        kept(state, path, text_of(child));
        undeclared_within(state, child, path, depth);
    }
    else if(!state.values.contains(declared))
        walk_undeclared(state, child, path, depth);
    else if(!child.first_attribute() && !carries_elements(child))
        kept(state, path, text_of(child));
    else
        walk_element(state, child, path, depth);
}

void walk_children(walk &state, pugi::xml_node element, const std::string &path, std::size_t depth)
{
    for(const pugi::xml_node child : element.children())
        if(child.type() == pugi::node_element)
            walk_child(state, child, joined(path, child.name()), depth);
}

void walk_element(walk &state, pugi::xml_node element, const std::string &path, std::size_t depth)
{
    const std::string where = path.empty() ? std::string(element.name()) : path;
    if(depth > deepest_element)
    {
        state.out.malformed.push_back("'" + where + "' is nested deeper than " + std::to_string(deepest_element) + " elements");
        return;
    }
    if(const std::optional<std::string> fault = structural_fault(element, where); fault)
        state.out.malformed.push_back(*fault);
    walk_attributes(state, element, path);
    walk_children(state, element, path, depth + 1);
}

std::string position_of(std::string_view source, std::ptrdiff_t offset)
{
    const std::string_view before = source.substr(0, static_cast<std::size_t>(std::max<std::ptrdiff_t>(offset, 0)));
    const std::size_t last_break  = before.rfind('\n');
    const std::size_t column      = last_break == std::string_view::npos ? before.size() + 1 : before.size() - last_break;
    return "line " + std::to_string(std::ranges::count(before, '\n') + 1) + ", column " + std::to_string(column);
}

std::optional<std::string> beside_the_root(const pugi::xml_document &held)
{
    const pugi::xml_node root = held.document_element();
    for(const pugi::xml_node piece : held.children())
    {
        if(piece.type() == pugi::node_element && piece != root)
            return "a second root element '" + std::string(piece.name()) + "' follows '" + root.name() + "'";
        if((piece.type() == pugi::node_pcdata || piece.type() == pugi::node_cdata) && !blank(piece.value()))
            return "the text '" + std::string(piece.value()) + "' stands outside the root element";
    }
    return std::nullopt;
}

}

expected<folding, error> folded(std::string_view source, const declaration &shape, const std::filesystem::path &from)
{
    pugi::xml_document held;
    const pugi::xml_parse_result read = held.load_buffer(source.data(), source.size(), pugi::parse_default);
    if(!read)
        return unexpected(
                error{error_code::malformed_source, "the configuration at " + from.string() + " does not parse at " + position_of(source, read.offset) + ": " + read.description()});
    if(const std::optional<std::string> beside = beside_the_root(held); beside)
        return unexpected(error{error_code::malformed_source, "the configuration at " + from.string() + " is not one document: " + *beside});

    const pugi::xml_node root = held.document_element();
    if(!shape.space().empty() && root.name() != shape.space())
        return unexpected(error{error_code::mismatched_space, "the configuration at " + from.string() + " has the root '" + root.name() + "', not '" + shape.space() + "'"});

    folding walked;
    walk state = walking(shape, walked);
    if(shape.space().empty())
        walk_children(state, held, std::string(), 0);
    else
        walk_element(state, root, std::string(), 0);
    return walked;
}

}
