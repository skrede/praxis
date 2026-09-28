#include "praxis/manipulator/model_file.h"

#include <threepp/math/Color.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/loaders/STLLoader.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <span>
#include <memory>
#include <string>
#include <optional>
#include <filesystem>
#include <system_error>

namespace praxis::manipulator {

namespace {

bool regular_file(const std::filesystem::path &candidate)
{
    std::error_code failed;

    return std::filesystem::is_regular_file(candidate, failed);
}

}

std::optional<std::filesystem::path> located_model(const std::filesystem::path &named, std::span<const std::filesystem::path> roots)
{
    if(named.empty())
        return std::nullopt;

    for(const std::filesystem::path &root : roots)
        if(regular_file(root / named))
            return root / named;

    if(regular_file(named))
        return named;

    return std::nullopt;
}

std::shared_ptr<threepp::Object3D> loaded_model(const std::filesystem::path &file)
{
    const threepp::STLLoader loader;
    const auto geometry = loader.load(file);
    if(geometry == nullptr)
        return nullptr;

    return threepp::Mesh::create(geometry, threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", threepp::Color::gray}}));
}

std::string loaded_model_line(const std::string &named, std::span<const std::filesystem::path> roots, bool holding)
{
    const std::optional<std::filesystem::path> file = located_model(named, roots);
    if(holding)
        return file ? file->string() : named;
    if(named.empty())
        return std::string();

    return file ? "none, " + file->string() + " could not be read" : "none, no search root holds " + named;
}

}
