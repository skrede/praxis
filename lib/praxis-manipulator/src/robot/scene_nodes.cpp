#include "robot/scene_nodes.h"

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <threepp/materials/Material.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>

#include <threepp/math/Euler.hpp>
#include <threepp/math/Color.hpp>
#include <threepp/math/Vector3.hpp>

#include <algorithm>
#include <filesystem>

namespace praxis::manipulator {

namespace {

const meios::material<double> *named_material(const meios::model<> &model, const meios::visual<> &visual)
{
    if(visual.material_inline && visual.material_inline->color)
        return &*visual.material_inline;
    if(!visual.material_ref)
        return nullptr;

    const auto it = std::ranges::find_if(model.materials, [&visual](const auto &candidate) { return candidate.name == *visual.material_ref; });

    return it != model.materials.end() && it->color ? &*it : nullptr;
}

std::shared_ptr<threepp::Material> resolve_material(const meios::model<> &model, const meios::visual<> &visual)
{
    const meios::material<double> *material = named_material(model, visual);
    if(!material)
        return nullptr;

    const auto &color = *material->color;
    const auto result = threepp::MeshStandardMaterial::create();
    result->color.setRGB(static_cast<float>(color.r), static_cast<float>(color.g), static_cast<float>(color.b));
    if(color.a < 1.0)
    {
        result->transparent = true;
        result->opacity     = static_cast<float>(color.a);
    }

    return result;
}

void paint(threepp::Object3D &object, const std::shared_ptr<threepp::Material> &material)
{
    object.traverseType<threepp::Mesh>([&material](threepp::Mesh &mesh) { mesh.setMaterial(material); });
}

std::shared_ptr<threepp::Material> collider_material()
{
    const auto material = threepp::MeshBasicMaterial::create();
    material->wireframe = true;
    material->color     = threepp::Color::white;

    return material;
}

std::filesystem::path mesh_root_of(const meios::link<> &link, const scene_robot_options &options)
{
    if(!options.mesh_root.empty())
        return options.mesh_root;
    if(link.origin_loc)
        return link.origin_loc->file.parent_path();

    return {};
}

void add_visuals(threepp::Object3D &node, const meios::model<> &model, const meios::link<> &link, mesh_library &meshes, const std::filesystem::path &mesh_root)
{
    for(const auto &visual : link.visuals)
    {
        auto group = threepp::Group::create();
        apply_origin(*group, visual.origin);

        if(auto geometry = meshes.build(visual.geom, mesh_root))
            group->add(geometry);
        if(auto material = resolve_material(model, visual))
            paint(*group, material);

        node.add(group);
    }
}

void add_colliders(threepp::Object3D &node, const meios::link<> &link, mesh_library &meshes, const std::filesystem::path &mesh_root)
{
    for(const auto &collision : link.collisions)
    {
        auto group                  = threepp::Group::create();
        group->userData["collider"] = true;
        apply_origin(*group, collision.origin);

        if(auto geometry = meshes.build(collision.geom, mesh_root))
        {
            group->add(geometry);
            paint(*geometry, collider_material());
        }

        node.add(group);
    }
}

}

void apply_origin(threepp::Object3D &object, const meios::transform<double> &origin)
{
    object.position.set(static_cast<float>(origin.translation.x), static_cast<float>(origin.translation.y), static_cast<float>(origin.translation.z));

    const threepp::Euler euler(static_cast<float>(origin.rotation.roll), static_cast<float>(origin.rotation.pitch), static_cast<float>(origin.rotation.yaw),
                               threepp::Euler::RotationOrders::ZYX);
    object.quaternion.setFromEuler(euler);
}

std::shared_ptr<threepp::Object3D> build_link_node(const meios::model<> &model, const meios::link<> &link, mesh_library &meshes, const scene_robot_options &options)
{
    auto node  = std::make_shared<threepp::Object3D>();
    node->name = link.name;

    const auto mesh_root = mesh_root_of(link, options);
    if(options.visuals)
        add_visuals(*node, model, link, meshes, mesh_root);
    if(options.colliders)
        add_colliders(*node, link, meshes, mesh_root);

    return node;
}

}
