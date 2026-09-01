#ifndef HPP_GUARD_PRAXIS_CONFIG_INSERTION_H
#define HPP_GUARD_PRAXIS_CONFIG_INSERTION_H

#include "praxis/config/writer.h"
#include "praxis/config/declaration.h"

#include <span>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::config {

// The element paths the leaves of `wanted` hang under that `source` does not carry, each named once
// however many keys share it and an ancestor always before what hangs below it. An instance of a
// collection `shape` declares is among them where, and only where, `wanted` names the identity of
// that instance, and of every one between it and the last the document already carries, as a value
// that is not empty; a key whose chain spells an instance nothing there names contributes nothing.
std::vector<std::string> absent_elements(const declaration &shape, std::string_view source, std::span<const edit> wanted);

// `source` with each of `absent` created as an empty element under the deepest ancestor it has, as
// that ancestor's last child and indented to match the siblings it joins. Every other byte is left
// exactly as it was, so what is grown this way can still be spliced by offset afterwards.
std::string with_elements(std::string source, std::span<const std::string> absent);

}

#endif
