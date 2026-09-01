praxis_add_module(config
    SOURCES
    include/praxis/config.h
    include/praxis/config/error.h
    include/praxis/config/store.h
    include/praxis/config/writer.h
    include/praxis/config/binding.h
    include/praxis/config/document.h
    include/praxis/config/declaration.h
    include/praxis/config/configurable.h
    src/engine.h
    src/locator.h
    src/announce.h
    src/key_path.h
    src/insertion.h
    src/read_back.h
    src/source_text.h
    src/error.cpp
    src/scalar.cpp
    src/space.cpp
    src/declaration.cpp
    src/document.cpp
    src/store.cpp
    src/announce.cpp
    src/key_path.cpp
    src/resolve.cpp
    src/template.cpp
    src/source_text.cpp
    src/locator.cpp
    src/insertion.cpp
    src/read_back.cpp
    src/writer.cpp
    src/binding.cpp
    PUBLIC_LIBS
    praxis::compat
    PRIVATE_LIBS
    nucleus::core
    nucleus::xml
    spdlog::spdlog
)

# The engine's XML module already carries the parser's objects into this target, so only its headers
# are missing, and they are taken as a directory rather than as a link edge: the parser sits in two
# export sets at once, and an exported target may not name one that does. A private include
# directory reaches no interface, so nothing is exported and the assertion below still holds.
target_include_directories(praxis_config PRIVATE $<TARGET_PROPERTY:pugixml::pugixml,INTERFACE_INCLUDE_DIRECTORIES>)

# The configuration engine, the parser behind it and the logging dependency are all linked privately
# and reach no published header, which is what this asserts over the whole interface closure.
praxis_assert_no_include_leak(praxis_config cartan ctrlpp meios threepp imgui spdlog nucleus pugixml)

set(PRAXIS_PUBLIC_DEPENDENCIES NONE)

# Linking a library costs this module the gate's widest branch, which asserted every dependency and
# every other target of the repository absent on its own. What is left is named here.
set(PRAXIS_CLOSURE_ABSENT cartan ctrlpp meios threepp imgui
    praxis_extension praxis_scheduler praxis_scene)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::config is told about a target it never named.
set_target_properties(praxis_config PROPERTIES EXPORT_NAME config)
