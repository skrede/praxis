#include "engine.h"
#include "key_path.h"

#include "praxis/config/store.h"

#include <pugixml.hpp>

#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <optional>
#include <algorithm>
#include <filesystem>
#include <string_view>
#include <system_error>

namespace praxis::config {
namespace {

struct segment_wise
{
    bool operator()(const std::string &left, const std::string &right) const
    {
        return std::ranges::lexicographical_compare(segments_of(left), segments_of(right));
    }
};

bool under_a_collection(const declaration &shape, const std::string &path)
{
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::collection && hangs_under(path, declared.path))
            return true;
    return false;
}

// A starter document names no instance of any collection, so a leaf that hangs under one carries no
// value there and is left out rather than invented at an ordinal nothing declared.
std::map<std::string, std::string, segment_wise> starter_values(const declaration &shape)
{
    std::map<std::string, std::string, segment_wise> named;
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::leaf && !under_a_collection(shape, declared.path))
            named.emplace(declared.path, declared.fallback);
    return named;
}

bool name_start(unsigned char byte)
{
    return byte >= 0x80 || (byte >= 'a' && byte <= 'z') || (byte >= 'A' && byte <= 'Z') || byte == '_' || byte == ':';
}

bool name_byte(unsigned char byte)
{
    return name_start(byte) || (byte >= '0' && byte <= '9') || byte == '-' || byte == '.';
}

bool xml_name(std::string_view name)
{
    return !name.empty() && name_start(static_cast<unsigned char>(name.front())) && std::ranges::all_of(name.substr(1), [](unsigned char byte) { return name_byte(byte); });
}

// The control bytes XML 1.0 cannot carry; a carriage return would read back as a line feed.
bool unwritable(std::string_view text)
{
    return std::ranges::any_of(text, [](unsigned char byte) { return byte < 0x20 && byte != '\t' && byte != '\n'; });
}

pugi::xml_node child_named(pugi::xml_node under, const std::string &name)
{
    const pugi::xml_node found = under.child(name.c_str());
    return found ? found : under.append_child(name.c_str());
}

// Whitespace alone goes in a CDATA section, the one place a parser keeps it as text.
std::optional<error> rendered_leaf(pugi::xml_node root, const std::string &path, const std::string &fallback)
{
    pugi::xml_node at = root;
    for(const std::string_view segment : segments_of(path))
    {
        if(!xml_name(segment))
            return error{error_code::malformed_source, "the leaf '" + path + "' cannot be written: '" + std::string(segment) + "' is not an XML name"};
        at = child_named(at, std::string(segment));
    }
    if(unwritable(fallback))
        return error{error_code::malformed_source, "the fallback of '" + path + "' carries a control character no XML document holds"};
    if(!fallback.empty())
        at.append_child(fallback.find_first_not_of(" \t\n") == std::string::npos ? pugi::node_cdata : pugi::node_pcdata).set_value(fallback.c_str());
    return std::nullopt;
}

expected<std::string, error> starter_text(const declaration &shape)
{
    if(!shape.space().empty() && !xml_name(shape.space()))
        return unexpected(error{error_code::malformed_source, "the space '" + shape.space() + "' is not an XML name, so no document can be rooted at it"});

    pugi::xml_document starter;
    const pugi::xml_node root = starter.append_child(shape.space().c_str());
    for(const auto &[path, fallback] : starter_values(shape))
        if(const std::optional<error> refused = rendered_leaf(root, path, fallback); refused)
            return unexpected(*refused);

    std::ostringstream text;
    starter.save(text, "  ", pugi::format_default, pugi::encoding_utf8);
    return text.str();
}

expected<void, error> landed(const std::string &rendered, const std::filesystem::path &target)
{
    const std::filesystem::path staging = target.parent_path() / (target.filename().string() + ".partial");

    std::ofstream out(staging, std::ios::trunc | std::ios::binary);
    out << rendered;
    out.close();
    if(!out)
    {
        std::error_code ignored;
        std::filesystem::remove(staging, ignored);
        return unexpected(error{error_code::unwritable_target, "the starter document for " + target.string() + " could not be written"});
    }

    std::error_code renaming;
    std::filesystem::rename(staging, target, renaming);
    if(renaming)
    {
        std::error_code ignored;
        std::filesystem::remove(staging, ignored);
        return unexpected(error{error_code::unwritable_target, "the starter document could not be put in place at " + target.string() + ": " + renaming.message()});
    }

    return {};
}

}

expected<void, error> write_template(const declaration &shape, const std::filesystem::path &target)
{
    if(const std::optional<error> refused = refused_as_declared(shape); refused)
        return unexpected(*refused);

    std::error_code probing;
    if(std::filesystem::exists(target, probing))
        return unexpected(error{error_code::unwritable_target, "there is already something at " + target.string()});

    const expected<std::string, error> rendered = starter_text(shape);
    if(!rendered)
        return unexpected(rendered.error());
    return landed(rendered.value(), target);
}

}
