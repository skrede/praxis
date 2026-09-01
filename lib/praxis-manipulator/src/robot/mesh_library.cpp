#include "robot/mesh_library.h"

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/SkinnedMesh.hpp>

#include <threepp/loaders/ModelLoader.hpp>

#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/geometries/SphereGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>

#include <threepp/math/Vector3.hpp>
#include <threepp/math/MathUtils.hpp>

#include <spdlog/spdlog.h>

#include <string>
#include <utility>
#include <variant>
#include <system_error>

namespace praxis::manipulator {

namespace {

threepp::Vector3 to_threepp(const meios::vector3<double> &vector)
{
    return {static_cast<float>(vector.x), static_cast<float>(vector.y), static_cast<float>(vector.z)};
}

std::shared_ptr<threepp::Object3D> build_box(const meios::box<double> &shape)
{
    auto object = threepp::Mesh::create(threepp::BoxGeometry::create(1, 1, 1));
    object->scale.copy(to_threepp(shape.size));

    return object;
}

std::shared_ptr<threepp::Object3D> build_cylinder(const meios::cylinder<double> &shape)
{
    const auto radius = static_cast<float>(shape.radius);
    auto object       = threepp::Mesh::create(threepp::CylinderGeometry::create(radius, radius, static_cast<float>(shape.length)));

    // A description's cylinders run along +Z, the renderer's along +Y.
    object->rotateX(threepp::math::PI / 2.f);

    return object;
}

std::filesystem::path resolve(const meios::mesh<> &shape, const std::filesystem::path &mesh_root)
{
    std::filesystem::path path = shape.resolved_path ? std::filesystem::path(*shape.resolved_path) : std::filesystem::path(shape.filename);
    if(path.is_relative() && !mesh_root.empty())
        return mesh_root / path;

    return path;
}

std::string cache_key(const std::filesystem::path &path)
{
    std::error_code ec;
    const auto canonical = std::filesystem::weakly_canonical(path, ec);

    return (ec ? path : canonical).string();
}

bool cloneable(threepp::Group &group)
{
    if(!group.animations.empty())
        return false;

    bool clean = true;
    group.traverseType<threepp::SkinnedMesh>([&clean](threepp::SkinnedMesh &) { clean = false; });

    return clean;
}

std::shared_ptr<threepp::Loader<threepp::Group>> make_model_loader()
{
    auto loader = std::make_shared<threepp::ModelLoader>();
    loader->setIgnoreUpDirection(true);

    return loader;
}

}

mesh_library::mesh_library()
        : m_loader(make_model_loader())
        , m_cache()
{
}

std::shared_ptr<threepp::Object3D> mesh_library::build(const meios::geometry<> &geometry, const std::filesystem::path &mesh_root)
{
    if(const auto *shape = std::get_if<meios::mesh<double>>(&geometry.shape))
    {
        auto object = load(*shape, mesh_root);
        if(object)
            object->scale.copy(to_threepp(shape->scale));

        return object;
    }
    if(const auto *shape = std::get_if<meios::box<double>>(&geometry.shape))
        return build_box(*shape);
    if(const auto *shape = std::get_if<meios::sphere<double>>(&geometry.shape))
        return threepp::Mesh::create(threepp::SphereGeometry::create(static_cast<float>(shape->radius)));
    if(const auto *shape = std::get_if<meios::cylinder<double>>(&geometry.shape))
        return build_cylinder(*shape);

    return nullptr;
}

std::shared_ptr<threepp::Group> mesh_library::load(const meios::mesh<> &shape, const std::filesystem::path &mesh_root)
{
    const auto path = resolve(shape, mesh_root);
    if(!m_loader || path.empty())
        return nullptr;

    const std::string key = cache_key(path);
    if(const auto it = m_cache.find(key); it != m_cache.end())
        return it->second->clone<threepp::Group>();

    auto loaded = m_loader->load(path);
    if(!loaded)
    {
        spdlog::warn("praxis: could not load mesh '{}'", path.string());
        return nullptr;
    }
    if(!cloneable(*loaded))
        return loaded;

    m_cache[key] = loaded;

    return loaded->clone<threepp::Group>();
}

}
