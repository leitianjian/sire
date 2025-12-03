cmake_minimum_required (VERSION 3.11)
 
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG main # Important: Specify a version or a commit hash here.
)
FetchContent_MakeAvailable(ftxui)

# add_subdirectory(demo_duan_cam_backend)
add_subdirectory(demo_middleware_collision)
add_subdirectory(demo_server_controller_sensor)
add_subdirectory(demo_server_integrator_dynamics)

# add_subdirectory(demo_server_sensor_root)
add_subdirectory(demo_simulation)
add_subdirectory(demo_main)

# add_custom_target(gentest COMMAND ${CMAKE_COMMAND} -E echo "\"$<$<CONFIG:Debug>:-g;-O0>,$<PLATFORM_ID>\"")
# add_custom_target(gentest COMMAND ${CMAKE_COMMAND} -E echo "\"${VCPKG_DLL_PREFIX}/boost_filesystem-vc${MSVC_TOOLSET_VERSION}-mt$<$<CONFIG:Debug>:-gd>-x64-${Boost_VERSION_MAJOR}_${Boost_VERSION_MINOR}.dll\"")

# add_subdirectory(demo_model_init)

# using ${TARGET_LIB_PATH} to find_package
# include(CMakeParseArguments)

# macro(INSTALL_DLL)
# if(WIN32)
# set(_OPTIONS_ARGS)
# set(_ONE_VALUE_ARGS DESTINATION)
# set(_MULTI_VALUE_ARGS PATHS CONFIGURATIONS)
# cmake_parse_arguments(_INSTALLDLL "${_OPTIONS_ARGS}" "${_ONE_VALUE_ARGS}" "${_MULTI_VALUE_ARGS}" ${ARGN})
#
# if(CMAKE_BUILD_TYPE IN_LIST _INSTALLDLL_CONFIGURATIONS)
# if(_INSTALLDLL_PATHS)
# # validate dll file is exist in path
# foreach(DLL_PATH ${_INSTALLDLL_PATHS})
# if(NOT EXISTS ${DLL_PATH})
# message(STATUS "${DLL_PATH} not exist!")
# continue()
# endif()
#
# get_filename_component(FILE_EXT ${DLL_PATH} LAST_EXT)
#
# if(NOT FILE_EXT STREQUAL ".dll")
# message(STATUS "${DLL_PATH} exist but not a dll ext file!")
# continue()
# endif()
#
# install(FILES ${DLL_PATH} CONFIGURATIONS ${_INSTALLDLL_CONFIGURATIONS} DESTINATION ${_INSTALLDLL_DESTINATION})
# endforeach()
# endif(_INSTALLDLL_PATHS)
# endif(CMAKE_BUILD_TYPE IN_LIST _INSTALLDLL_CONFIGURATIONS)
# endif(WIN32)
# endmacro()

# macro(INSTALL_ARIS_FCL_DLL)
# if(WIN32)
# set(_OPTIONS_ARGS)
# set(_ONE_VALUE_ARGS DESTINATION)
# set(_MULTI_VALUE_ARGS)
# cmake_parse_arguments(_INSTALLDLL "${_OPTIONS_ARGS}" "${_ONE_VALUE_ARGS}" "${_MULTI_VALUE_ARGS}" ${ARGN})

# install_dll(
# PATHS
# "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE_LOWER}/aris_lib.dll"
# "${TARGET_HPP_FCL_PATH}/bin/coal.dll"
# DESTINATION
# ${_INSTALLDLL_DESTINATION}
# )
# endif(WIN32)
# endmacro()