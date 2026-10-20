praxis_add_test(solver_test.cpp LIBS praxis::optimization)
praxis_add_test(solver_row_test.cpp LIBS praxis::optimization praxis::evaluation praxis_test_support)
praxis_add_test(projected_solver_test.cpp LIBS praxis::optimization)
