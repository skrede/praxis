praxis_add_module(scheduler
    SOURCES
    include/praxis/scheduler/task.h
    include/praxis/scheduler/clock.h
    include/praxis/scheduler/strand.h
    include/praxis/scheduler/overrun.h
    include/praxis/scheduler/snapshot.h
    include/praxis/scheduler/rejection.h
    include/praxis/scheduler/scheduler.h
    include/praxis/scheduler/ownership.h
    src/pool.h
    src/queue.h
    src/marker.h
    src/deadline.h
    src/pool.cpp
    src/task.cpp
    src/queue.cpp
    src/strand.cpp
    src/marker.cpp
    src/deadline.cpp
    src/scheduler.cpp
    src/ownership.cpp
    src/pool_waiting.cpp
    src/pool_service.cpp
    src/pool_receipts.cpp
    src/pool_admission.cpp
    PUBLIC_LIBS
    praxis::compat
)

praxis_assert_no_include_leak(praxis_scheduler cartan ctrlpp meios threepp imgui spdlog)

set(PRAXIS_PUBLIC_DEPENDENCIES NONE)

# Linking a library costs this module the gate's widest branch, which asserted every dependency and
# every other target of the repository absent on its own. What is left is this list, so the two
# targets that branch used to cover are named here.
set(PRAXIS_CLOSURE_ABSENT cartan ctrlpp meios threepp imgui spdlog praxis_extension praxis_scene)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::scheduler is told about a target it never named.
set_target_properties(praxis_scheduler PROPERTIES EXPORT_NAME scheduler)
