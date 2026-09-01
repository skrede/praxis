set(PRAXIS_WARNING_FLAGS
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion;-Wsign-conversion;-Wold-style-cast;-Wnon-virtual-dtor;-Woverloaded-virtual>
)

function(praxis_target_warnings TARGET)
    target_compile_options(${TARGET} PRIVATE ${PRAXIS_WARNING_FLAGS})
    target_compile_definitions(${TARGET} PRIVATE $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>)
endfunction()
