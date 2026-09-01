#include "engine.h"
#include "key_path.h"

#include "praxis/config/store.h"

#include <nucleus/config.h>
#include <nucleus/config_space.h>

#include <nucleus/xml/xml_emitter.h>

#include <map>
#include <string>
#include <fstream>
#include <filesystem>
#include <system_error>

namespace praxis::config {
namespace {

bool under_a_collection(const declaration &shape, const std::string &path)
{
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::collection && hangs_under(path, declared.path))
            return true;
    return false;
}

// A starter document names no instance of any collection, so a leaf that hangs under one carries no
// value there and is left out rather than invented at an ordinal nothing declared.
std::map<std::string, std::string> starter_values(const declaration &shape)
{
    std::map<std::string, std::string> named;
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::leaf && !under_a_collection(shape, declared.path))
            named.emplace(declared.path, declared.fallback);
    return named;
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
    const expected<nucleus::config_space, error> space = sealed_space(shape);
    if(!space)
        return unexpected(space.error());

    std::error_code probing;
    if(std::filesystem::exists(target, probing))
        return unexpected(error{error_code::unwritable_target, "there is already something at " + target.string()});

    const nucleus::expected<nucleus::config, nucleus::error> filled = nucleus::config::from_values(starter_values(shape));
    if(!filled)
        return unexpected(translated(filled.error()));

    const nucleus::expected<std::string, nucleus::error> rendered = nucleus::xml::render_document(filled.value(), space.value(), shape.space());
    if(!rendered)
        return unexpected(translated(rendered.error()));

    return landed(rendered.value(), target);
}

}
