praxis_add_test(tolerance_test.cpp LIBS praxis::rigid_motion praxis::trajectory)

praxis_add_test(comparators_test.cpp LIBS praxis::evaluation)
praxis_add_test(comparators_logarithm_test.cpp LIBS praxis::evaluation)
praxis_add_test(refusal_agreement_test.cpp LIBS praxis::evaluation)
praxis_add_test(slot_fold_test.cpp LIBS praxis::evaluation)

praxis_add_test(generation_test.cpp LIBS praxis::evaluation)
praxis_add_test(generation_spread_test.cpp LIBS praxis::evaluation)
praxis_add_test(generation_reproducibility_test.cpp LIBS praxis::evaluation)

# A drawn transform is held against the membership the tree's own paths refuse a non-member to,
# rather than against a predicate written a second time here.
praxis_add_test(generation_member_test.cpp LIBS praxis::evaluation praxis::trajectory)

# The facility is generic and the cases that drive it need a capability to drive, so these two name
# the extension whose slots they compare.
praxis_add_test(evaluation_contract_test.cpp LIBS praxis::rigid_motion)
praxis_add_test(rigid_motion_ledger_test.cpp LIBS praxis::evaluation praxis::rigid_motion)
praxis_add_test(tolerance_selection_test.cpp HEADERS bent_rigid_motion.h LIBS praxis::evaluation praxis::rigid_motion)
praxis_add_test(bend_control_test.cpp HEADERS bent_answer.h bent_frame.h bent_screw.h bend_visibility.h bent_rigid_motion.h LIBS praxis::evaluation praxis::rigid_motion)
praxis_add_test(honesty_test.cpp LIBS praxis::evaluation praxis::rigid_motion)
