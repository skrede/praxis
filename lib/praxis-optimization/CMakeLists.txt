praxis_add_module(optimization
    SOURCES
    include/praxis/optimization.h
    include/praxis/optimization/slots.h
    include/praxis/optimization/solver.h
    include/praxis/optimization/problem.h
    include/praxis/optimization/evaluation.h
    include/praxis/optimization/capabilities.h
    include/praxis/optimization/baseline/solver.h
    include/praxis/optimization/projected/solver.h
    src/evaluation.cpp
    src/descriptors.cpp
    src/inert_solver.cpp
    src/evaluation_tables.h
    src/evaluation_problems.cpp
    src/baseline/solver.cpp
    src/baseline/capabilities.cpp
    src/projected/solver.cpp
    src/projected/step_constants.h
    src/projected/periodic_bounds.h
    src/projected/periodic_bounds.cpp
    PUBLIC_LIBS
    praxis::extension
    praxis::evaluation
    Eigen3::Eigen
    PRIVATE_LIBS
    spdlog::spdlog
)

# The logging dependency is linked privately and the model library not at all; neither may reach the
# published interface.
praxis_assert_no_include_leak(praxis_optimization meios spdlog)

set(PRAXIS_PUBLIC_DEPENDENCIES Eigen3)

# The link-isolation gate reads this list and fails on the model library reaching this target's closure.
set(PRAXIS_CLOSURE_ABSENT meios)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::optimization is told about a target it never named.
set_target_properties(praxis_optimization PROPERTIES EXPORT_NAME optimization)
