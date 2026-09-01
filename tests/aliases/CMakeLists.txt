# There is nothing to run: the target resolving every alias and linking every module is the
# assertion, so it registers no test.
add_executable(praxis_alias_link alias_link.cpp)

praxis_target_warnings(praxis_alias_link)

set(praxis_alias_libs praxis::praxis praxis::extensions)
foreach (module IN LISTS PRAXIS_CORE_MODULES PRAXIS_EXTENSIONS)
    list(APPEND praxis_alias_libs praxis::${module})
endforeach ()

target_link_libraries(praxis_alias_link PRIVATE ${praxis_alias_libs})
