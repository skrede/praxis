praxis_add_module(extension
    SOURCES
    include/praxis/extension.h
    include/praxis/extension/refusal.h
    include/praxis/extension/coverage.h
    include/praxis/extension/slot_set.h
    include/praxis/extension/descriptor.h
    include/praxis/extension/held_handle.h
    src/coverage.cpp
    src/held_handle.cpp
    PUBLIC_LIBS
    praxis::compat
)

praxis_assert_no_include_leak(praxis_extension cartan ctrlpp meios threepp imgui spdlog)

set(PRAXIS_PUBLIC_DEPENDENCIES NONE)

# Linking a library costs this module the gate's widest branch, which asserted every dependency and
# every other target of the repository absent on its own. What is left is this list, so the two
# targets that branch used to cover are named here.
set(PRAXIS_CLOSURE_ABSENT cartan ctrlpp meios threepp imgui spdlog praxis_scene praxis_scheduler)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::extension is told about a target it never named.
set_target_properties(praxis_extension PROPERTIES EXPORT_NAME extension)
