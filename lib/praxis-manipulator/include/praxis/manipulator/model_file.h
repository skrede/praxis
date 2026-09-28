#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_FILE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_FILE_H

#include <threepp/core/Object3D.hpp>

#include <span>
#include <memory>
#include <string>
#include <optional>
#include <filesystem>

namespace praxis::manipulator {

// The first root, in the order given, holding `named` as a regular file, then `named` itself where it
// is one. A blank name locates nothing.
std::optional<std::filesystem::path> located_model(const std::filesystem::path &named, std::span<const std::filesystem::path> roots);

// The STL at `file` as a mesh, or null where the loader reads no geometry from it.
std::shared_ptr<threepp::Object3D> loaded_model(const std::filesystem::path &file);

// What a window naming `named` says it loaded: while it holds a mesh, the file located or else the
// name; holding none, why not, and nothing at all for a blank name.
std::string loaded_model_line(const std::string &named, std::span<const std::filesystem::path> roots, bool holding);

}

#endif
