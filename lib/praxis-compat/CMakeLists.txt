praxis_add_module(compat
    SOURCES
    include/praxis/compat/expected.h
    include/praxis/compat/detail/expected.h
    include/praxis/compat/detail/callable.h
    src/callable.cpp
)

praxis_assert_no_include_leak(praxis_compat cartan ctrlpp meios threepp imgui spdlog)

# What this module carries into a consumer's build through its own published headers. The
# link-isolation gate reads the list and stops on a public link to anything it does not name;
# a module that publishes no dependency at all writes NONE.
set(PRAXIS_PUBLIC_DEPENDENCIES NONE)

# This module links nothing at all, and the link-isolation gate holds its closure to that.
set(PRAXIS_CLOSURE_ABSENT cartan ctrlpp meios threepp imgui spdlog)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::compat is told about a target it never named. The export calls are in the umbrella
# listfile, which is the first point at which every exported target exists.
set_target_properties(praxis_compat PROPERTIES EXPORT_NAME compat)
