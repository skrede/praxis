include(FetchContent)

if (NOT DEFINED MEIOS_CMAKE_FETCH_DEPS)
    set(MEIOS_CMAKE_FETCH_DEPS ON)
endif ()
if (NOT DEFINED FETCHCONTENT_TRY_FIND_PACKAGE_MODE)
    set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
endif ()

# SOURCE_SUBDIR names a directory that does not exist, so the source is downloaded but Eigen's own
# listfile is never processed and none of its tree is configured; the target the headers travel on
# is therefore created here by hand.
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL
    SOURCE_SUBDIR this-directory-does-not-exist
    SYSTEM
    FIND_PACKAGE_ARGS 3.4 CONFIG NAMES Eigen3 GLOBAL
)
FetchContent_MakeAvailable(Eigen3)

if (NOT TARGET Eigen3::Eigen)
    add_library(praxis_eigen_headers INTERFACE)
    add_library(Eigen3::Eigen ALIAS praxis_eigen_headers)
    target_include_directories(praxis_eigen_headers SYSTEM INTERFACE
        "$<BUILD_INTERFACE:${eigen3_SOURCE_DIR}>"
    )
    # NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
    # writing praxis::eigen_headers is told about a target it never named.
    set_target_properties(praxis_eigen_headers PROPERTIES EXPORT_NAME eigen_headers)

    # Eigen's own switch: including an LGPL-licensed Eigen header while it is defined is a compile
    # error. It is confined to the build interface, so it holds over everything this tree compiles
    # and is not imposed on a consumer of the installed library. An alias cannot carry a definition,
    # so the fetched branch names the target underneath and the found branch names the imported
    # target, which FIND_PACKAGE_ARGS made global.
    target_compile_definitions(praxis_eigen_headers INTERFACE $<BUILD_INTERFACE:EIGEN_MPL2_ONLY>)
else ()
    target_compile_definitions(Eigen3::Eigen INTERFACE $<BUILD_INTERFACE:EIGEN_MPL2_ONLY>)
endif ()

# The revision and the install setting are decided here because meios, nucleus and cartan each
# acquire pugixml themselves at differing revisions and with differing install settings, leaving the
# outcome to whichever of them happens to be declared first. Install rules stay enabled because
# nucleus link-references pugixml from an export set it does not gate, and an export set cannot
# generate against a dependency that installs nothing.
set(PUGIXML_BUILD_TESTS OFF)
set(PUGIXML_INSTALL ON)

FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG c8033ce9d039e7f9d134877c363397b3cfe20816  # pugixml v1.16
    SYSTEM
    FIND_PACKAGE_ARGS 1.16 NAMES pugixml GLOBAL
)
FetchContent_MakeAvailable(pugixml)

include(${CMAKE_CURRENT_LIST_DIR}/scene_dependencies.cmake)

if (PRAXIS_BUILD_TESTS)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.15.3
    )
    FetchContent_MakeAvailable(Catch2)
endif ()

FetchContent_Declare(
    cartan
    GIT_REPOSITORY https://github.com/skrede/cartan.git
    GIT_TAG 7c69e82c885f56ed694dbba13b9a199e061b3745  # milestone/v0.4.3
)
FetchContent_MakeAvailable(cartan)

FetchContent_Declare(
    ctrlpp
    GIT_REPOSITORY https://github.com/skrede/ctrlpp.git
    GIT_TAG 91b0d3f282c89c36ad50abd10473d12f63c851c6  # milestone/v0.3.6
)
FetchContent_MakeAvailable(ctrlpp)

FetchContent_Declare(
    meios
    GIT_REPOSITORY https://github.com/skrede/meios.git
    GIT_TAG a7018a8c1905aa5634b886ae9a6e644462f8ff3d  # milestone/v0.2.2
    SYSTEM
)
FetchContent_MakeAvailable(meios)

FetchContent_Declare(
    nucleus
    GIT_REPOSITORY https://github.com/skrede/nucleus.git
    GIT_TAG 94d760cda2dba91baaec0789062f876f5d29c204  # milestone/v0.4.2
)
FetchContent_MakeAvailable(nucleus)

meios_declare_resource(
    NAME kuka_experimental
    GITHUB ros-industrial/kuka_experimental
    REF 8d9292b04a22628b1b78d989e2ddd3abb913bf92
    HASH SHA256=02f299d967868fb32022617429ddede3a197fb8f18e09b88f88732e11a78bf79
)

meios_declare_resource(
    NAME ur_description
    GITHUB UniversalRobots/Universal_Robots_ROS2_Description
    REF 4.3.1
    SPARSE_PATHS urdf config meshes/ur3e
)
