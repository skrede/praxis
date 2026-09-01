praxis_add_module(evaluation
    SOURCES
    include/praxis/evaluation.h
    include/praxis/evaluation/report.h
    include/praxis/evaluation/residual.h
    include/praxis/evaluation/tolerance.h
    include/praxis/evaluation/generation.h
    include/praxis/evaluation/comparators.h
    include/praxis/evaluation/slot_evaluation.h
    src/evaluate.cpp
    src/tolerance.cpp
    src/generation.cpp
    src/comparators.cpp
    src/generation_members.cpp
    src/comparators_logarithm.cpp
    PUBLIC_LIBS
    praxis::extension
    Eigen3::Eigen
)

praxis_assert_no_include_leak(praxis_evaluation cartan ctrlpp meios spdlog)

set(PRAXIS_PUBLIC_DEPENDENCIES Eigen3)

# A comparator is built from linear algebra and nothing else, so the solver the reference bindings
# delegate to, the control library and the model library stay out of this closure entirely; the
# link-isolation gate reads the list and fails on any of them reaching it.
set(PRAXIS_CLOSURE_ABSENT cartan ctrlpp meios)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::evaluation is told about a target it never named.
set_target_properties(praxis_evaluation PROPERTIES EXPORT_NAME evaluation)
