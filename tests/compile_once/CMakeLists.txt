# Each probe is guarded on the module it links, so removing an entry from the umbrella's list
# removes that probe rather than failing the configure on a target that no longer exists.

# There is nothing to run: the target compiling is the assertion, so it registers no test. It links
# the manipulator extension alone, so a contract that needs anything else fails here.
if (TARGET praxis_manipulator)
    add_library(praxis_compile_once_manipulator_probe OBJECT
        manipulator_isolation.cpp
        student_solver.cpp
        bound_preset.cpp
    )

    praxis_target_warnings(praxis_compile_once_manipulator_probe)
    target_link_libraries(praxis_compile_once_manipulator_probe PRIVATE praxis::manipulator)
endif ()

# The same assertion for the evaluation module, which names no domain and links no extension: an
# extension header reaching a consumer of this module fails here.
if (TARGET praxis_evaluation)
    add_library(praxis_compile_once_evaluation_probe OBJECT
        evaluation_isolation.cpp
    )

    praxis_target_warnings(praxis_compile_once_evaluation_probe)
    target_link_libraries(praxis_compile_once_evaluation_probe PRIVATE praxis::evaluation)
endif ()

# The same assertion for the rigid-motion extension, on its own target so that it links that module
# alone: a contract of its that needs anything else fails here.
if (TARGET praxis_rigid_motion)
    add_library(praxis_compile_once_rigid_motion_probe OBJECT
        rigid_motion_isolation.cpp
    )

    praxis_target_warnings(praxis_compile_once_rigid_motion_probe)
    target_link_libraries(praxis_compile_once_rigid_motion_probe PRIVATE praxis::rigid_motion)
endif ()

# The same assertion for the trajectory extension. The control library is linked privately by that
# module, so its include directory must not reach here either: that is the live proof the private
# link stops at the implementation.
if (TARGET praxis_trajectory)
    add_library(praxis_compile_once_trajectory_probe OBJECT
        trajectory_isolation.cpp
    )

    praxis_target_warnings(praxis_compile_once_trajectory_probe)
    target_link_libraries(praxis_compile_once_trajectory_probe PRIVATE praxis::trajectory)
endif ()

# The per-target include-leak assertion walks a target's transitive interface, and every extension
# legitimately links the scheduler because its presets register tasks, so it cannot answer the
# per-header question. This probe links no praxis target at all and its include path is built here:
# the Foundation's two published roots, the three extensions', and the dependencies those headers
# publish. A slot table reaching the scheduler's or the scene's tree is then a missing header.
if (TARGET praxis_compat AND TARGET praxis_extension AND TARGET praxis_rigid_motion AND TARGET praxis_trajectory AND TARGET praxis_manipulator)
    set(praxis_slot_binding_includes
        ${PROJECT_SOURCE_DIR}/lib/praxis-compat/include
        ${PROJECT_SOURCE_DIR}/lib/praxis-extension/include
    )
    foreach (module IN LISTS PRAXIS_EXTENSIONS)
        list(APPEND praxis_slot_binding_includes ${PROJECT_SOURCE_DIR}/lib/praxis-${module}/include)
    endforeach ()

    # The linear-algebra library is published by every extension's vocabulary header and the model
    # library by the one that reads a robot description, so both are on the path the headers need.
    # They arrive as system directories, which is what linking their targets would have done and what
    # keeps the project's warning set pointed at the project.
    set(praxis_slot_binding_dependencies "")
    foreach (dependency IN ITEMS Eigen3::Eigen meios::urdf)
        _praxis_interface_include_closure(${dependency} praxis_dependency_includes)
        list(APPEND praxis_slot_binding_dependencies ${praxis_dependency_includes})
    endforeach ()

    add_library(praxis_compile_once_slot_binding_probe OBJECT
        slot_binding_isolation.cpp
    )

    praxis_target_warnings(praxis_compile_once_slot_binding_probe)
    target_compile_features(praxis_compile_once_slot_binding_probe PRIVATE cxx_std_20)
    target_include_directories(praxis_compile_once_slot_binding_probe PRIVATE ${praxis_slot_binding_includes})
    target_include_directories(praxis_compile_once_slot_binding_probe SYSTEM PRIVATE ${praxis_slot_binding_dependencies})
endif ()

# The machinery links no first-party module at all, so its probe asserts the wider footprint: no
# other module of this repository reaches it either.
if (TARGET praxis_extension)
    add_library(praxis_compile_once_extension_probe OBJECT
        extension_isolation.cpp
    )

    praxis_target_warnings(praxis_compile_once_extension_probe)
    target_link_libraries(praxis_compile_once_extension_probe PRIVATE praxis::extension)
endif ()
