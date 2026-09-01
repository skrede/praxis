#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_MESH_LIBRARY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_MESH_LIBRARY_H

#include <threepp/objects/Group.hpp>

#include <threepp/loaders/Loader.hpp>

#include <threepp/core/Object3D.hpp>

#include <meios/model.h>

#include <memory>
#include <string>
#include <filesystem>
#include <unordered_map>

namespace praxis::manipulator {

// Turns a description's geometry into renderer objects, keeping one loaded copy of each mesh file
// and handing out clones. A mesh carrying animations or skinning is not cloneable and is handed
// over whole, so a description referencing one twice loads it twice.
class mesh_library
{
public:
    mesh_library();

    std::shared_ptr<threepp::Object3D> build(const meios::geometry<> &geometry, const std::filesystem::path &mesh_root);

private:
    std::shared_ptr<threepp::Loader<threepp::Group>> m_loader;
    std::unordered_map<std::string, std::shared_ptr<threepp::Group>> m_cache;

    std::shared_ptr<threepp::Group> load(const meios::mesh<> &shape, const std::filesystem::path &mesh_root);
};

}

#endif
