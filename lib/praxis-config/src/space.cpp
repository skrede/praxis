#include "engine.h"

#include <any>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <typeindex>
#include <string_view>

namespace praxis::config {
namespace {

std::pair<std::string, std::string> split(const std::string &path)
{
    const std::size_t cut = path.rfind('/');
    if(cut == std::string::npos)
        return {std::string(), path};
    return {path.substr(0, cut), path.substr(cut + 1)};
}

nucleus::anchor anchor_of(const std::string &parent)
{
    return parent.empty() ? nucleus::anchor::root() : nucleus::anchor::keyspace(parent);
}

nucleus::expected<std::any, std::string> converted_flag(std::string_view text)
{
    if(const std::optional<bool> value = as_flag(text); value)
        return std::any(*value);
    return nucleus::unexpected(std::string("not a flag: ") + std::string(text));
}

nucleus::expected<std::any, std::string> converted_real(std::string_view text)
{
    if(const std::optional<double> value = as_real(text); value)
        return std::any(*value);
    return nucleus::unexpected(std::string("not a real number: ") + std::string(text));
}

nucleus::expected<std::any, std::string> converted_integer(std::string_view text)
{
    if(const std::optional<std::int64_t> value = as_integer(text); value)
        return std::any(*value);
    return nucleus::unexpected(std::string("not an integer: ") + std::string(text));
}

nucleus::expected<std::any, std::string> converted_text(std::string_view text)
{
    return std::any(std::string(text));
}

void apply_kind(nucleus::schema_element &leaf, field_kind kind)
{
    switch(kind)
    {
        case field_kind::flag:
            leaf.type_identity = std::type_index(typeid(bool));
            leaf.converter     = converted_flag;
            return;
        case field_kind::real:
            leaf.type_identity = std::type_index(typeid(double));
            leaf.converter     = converted_real;
            return;
        case field_kind::integer:
            leaf.type_identity = std::type_index(typeid(std::int64_t));
            leaf.converter     = converted_integer;
            return;
        case field_kind::text:
        case field_kind::choice:
            leaf.type_identity = std::type_index(typeid(std::string));
            leaf.converter     = converted_text;
            return;
    }
}

std::vector<nucleus::schema_element> elements_of(const node &declared)
{
    const std::pair<std::string, std::string> where = split(declared.path);
    const nucleus::anchor at                        = anchor_of(where.first);

    if(declared.shape == node_kind::group)
        return {nucleus::element(where.second, at)};

    if(declared.shape == node_kind::collection)
        return {nucleus::merging(nucleus::repeated_element(where.second, at), nucleus::merge_mode::replace_by_key),
                nucleus::element(declared.identity, nucleus::anchor::keyspace(declared.path))};

    nucleus::schema_element leaf = declared.allowed.empty() ? nucleus::element(where.second, at) : nucleus::enum_element(where.second, at, declared.allowed);
    apply_kind(leaf, declared.kind);
    return {leaf};
}

// Pooling the identity field is what makes it required on every instance and distinct among them;
// the keyed merge mode on its own only keys the composition across layers.
nucleus::identity_group_spec pooling(const node &declared)
{
    const std::pair<std::string, std::string> where = split(declared.path);
    return nucleus::identity_group(declared.path, anchor_of(where.first)).members({where.second}).field(declared.identity);
}

}

expected<nucleus::config_space, error> sealed_space(const declaration &shape)
{
    nucleus::config_space_builder builder;
    builder.name(shape.space());

    for(const node &declared : shape.nodes())
    {
        for(const nucleus::schema_element &entry : elements_of(declared))
            if(const nucleus::registration_result done = builder.register_element(entry); !done)
                return unexpected(translated(done.error()));

        if(declared.shape != node_kind::collection)
            continue;
        if(split(declared.path).first.empty())
            return unexpected(error{error_code::malformed_source,
                                    "the collection '" + declared.path +
                                            "' hangs directly under the root, where its identity field cannot "
                                            "be pooled; declare it under a group"});
        if(const nucleus::registration_result done = builder.register_identity_group(pooling(declared)); !done)
            return unexpected(translated(done.error()));
    }

    return builder.build();
}

}
