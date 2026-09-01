# The self-check links no module and declares its own rejection, so nothing else in this repository
# can make it compile: it is the one probe whose failure depends on no library in the tree.
praxis_add_failing_compile(harness_self_check.cpp DIAGNOSTIC "deleted function")

# One probe per accessor, so reverting a single deleted overload turns exactly one test red. Each is
# guarded on the module it links, so removing an entry from the umbrella's list removes its probes
# rather than failing the configure on a target that no longer exists.
# The refusal channel's type carries the attribute, and the project promotes no warning to an error
# anywhere else, so the drop is a rejection only where these flags are on. GCC classifies a discarded
# attributed return under unused-result and Clang under unused-value; each accepts the other's flag,
# and passing only one leaves the probe inert on the family that does not use it.
# One probe per attribute: the void specialization carries its own and inherits nothing from the
# primary template, so a single probe dropping both results would stay red with either one deleted.
# The attribute's own name is not the diagnostic to match: it is written in the source line these
# compilers echo under the caret, so a syntax error on the class declaration carrying it satisfies
# the match while probing nothing. Only wording the compiler composes discriminates. GCC writes
# "ignoring returned value", Clang "ignoring return value" and MSVC "discarding return value", and
# GCC quotes the attribute with locale-dependent characters, so the word order and the quoting are
# both unusable.
if (TARGET praxis_extension)
    praxis_add_failing_compile(discarded_refusal.cpp DIAGNOSTIC "ignoring return|discarding return" LIBS praxis::extension
        OPTIONS
        $<$<CXX_COMPILER_ID:MSVC>:/WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror=unused-result>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror=unused-value>)

    praxis_add_failing_compile(discarded_valueless_refusal.cpp DIAGNOSTIC "ignoring return|discarding return" LIBS praxis::extension
        OPTIONS
        $<$<CXX_COMPILER_ID:MSVC>:/WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror=unused-result>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror=unused-value>)
endif ()

if (TARGET praxis_scheduler)
    praxis_add_failing_compile(sample_time_transposed.cpp DIAGNOSTIC "cannot convert|no viable conversion" LIBS praxis::scheduler)
    praxis_add_failing_compile(step_delta_transposed.cpp DIAGNOSTIC "cannot convert|no viable conversion" LIBS praxis::scheduler)

    # The absent member is what the compiler composes, and the two families word it differently; the
    # returned reference is refused by an assertion this library writes, so its own text is the one
    # string no other failure in the translation unit can produce.
    praxis_add_failing_compile(strand_owned_escape.cpp DIAGNOSTIC "no member named|is not a member" LIBS praxis::scheduler)
    praxis_add_failing_compile(strand_owned_return.cpp DIAGNOSTIC "must not leave its gate by reference" LIBS praxis::scheduler)
endif ()

# Registration is a non-static member function and the registry publishes no instance of its own, so
# a namespace-scope initializer has nothing to register into and the call is the rejection. The
# families word the missing object differently, and the source line echoed under the caret names only
# the call, so the wording is the one text no other failure in this unit composes.
if (TARGET praxis_scene)
    praxis_add_failing_compile(namespace_scope_preset_registrar.cpp DIAGNOSTIC "without object|without an object|illegal call of non-static member function" LIBS praxis::scene)
endif ()

if (TARGET praxis_evaluation)
    praxis_add_failing_compile(evaluation_view_over_temporary.cpp DIAGNOSTIC "deleted function" LIBS praxis::evaluation)
endif ()

if (TARGET praxis_rigid_motion)
    praxis_add_failing_compile(rigid_motion_frame_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::rigid_motion)
    praxis_add_failing_compile(rigid_motion_screw_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::rigid_motion)

    # One probe per required constructor argument, so restoring a single default turns exactly one
    # test red: a roster given no noun to generate names from, and one given no way to read and write
    # the selection it stands on. The construction is the only rejected expression in either unit, so
    # overload resolution finding no constructor is the one wording no other failure there composes.
    # GCC writes "no matching function for call to", Clang "no matching constructor" and MSVC
    # "no overloaded function takes N arguments".
    praxis_add_failing_compile(rigid_motion_unnamed_roster.cpp DIAGNOSTIC "no matching|no overloaded function" LIBS praxis::rigid_motion)
    praxis_add_failing_compile(rigid_motion_unrouted_roster.cpp DIAGNOSTIC "no matching|no overloaded function" LIBS praxis::rigid_motion)
endif ()

if (TARGET praxis_trajectory)
    praxis_add_failing_compile(trajectory_path_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::trajectory)
    praxis_add_failing_compile(trajectory_trajectory_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::trajectory)
    praxis_add_failing_compile(trajectory_time_scaling_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::trajectory)
    praxis_add_failing_compile(trajectory_pose_trajectory_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::trajectory)
endif ()

if (TARGET praxis_manipulator)
    praxis_add_failing_compile(manipulator_robot_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_motion_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_modeling_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_forward_kinematics_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_differential_kinematics_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_inverse_kinematics_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_trajectory_view.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_evaluation_view_over_temporary.cpp DIAGNOSTIC "deleted function" LIBS praxis::manipulator)
    praxis_add_failing_compile(capability_view_mismatch.cpp DIAGNOSTIC "no matching|no overloaded function" LIBS praxis::manipulator)

    # One probe per solve. A bound solve is handed the maps its own mathematics reads and the chain it
    # solves over, and none of those carries a solve, so a slot body asking one of them for one names
    # a member that is not there. The families word an absent member differently, and the call is the
    # only expression in each unit, so that wording is the one text no other failure there composes.
    praxis_add_failing_compile(manipulator_solver_reentry.cpp DIAGNOSTIC "no member named|is not a member" LIBS praxis::manipulator)
    praxis_add_failing_compile(manipulator_analytic_solver_reentry.cpp DIAGNOSTIC "no member named|is not a member" LIBS praxis::manipulator)
endif ()
