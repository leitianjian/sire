cmake_minimum_required (VERSION 3.11)
 
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG main # Important: Specify a version or a commit hash here.
)
FetchContent_MakeAvailable(ftxui)

string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)

if(DEFINED VCPKG_INSTALLED_DIR)
  if(CMAKE_BUILD_TYPE_LOWER STREQUAL "debug")
    set(VCPKG_DLL_PREFIX ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/bin)
  elseif(CMAKE_BUILD_TYPE_LOWER STREQUAL "release")
    set(VCPKG_DLL_PREFIX ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin)
  endif()
endif(DEFINED VCPKG_INSTALLED_DIR)

macro(INSTALL_DLL)
  if(WIN32)
    set(_OPTIONS_ARGS)
    set(_ONE_VALUE_ARGS DESTINATION)
    set(_MULTI_VALUE_ARGS)
    cmake_parse_arguments(_INSTALLDLL "${_OPTIONS_ARGS}" "${_ONE_VALUE_ARGS}" "${_MULTI_VALUE_ARGS}" ${ARGN})

    install(
      FILES
        "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE_LOWER}/aris_lib.dll"
        "${TARGET_HPP_FCL_PATH}/bin/coal.dll"
      DESTINATION
        ${_INSTALLDLL_DESTINATION}
    )

    if(DEFINED VCPKG_DLL_PREFIX)
      install(
        FILES
          "${VCPKG_DLL_PREFIX}/assimp-vc${MSVC_TOOLSET_VERSION}-mt$<$<CONFIG:Debug>:d>.dll"
          "${VCPKG_DLL_PREFIX}/zlib$<$<CONFIG:Debug>:d>1.dll"
          "${VCPKG_DLL_PREFIX}/pugixml.dll"
          "${VCPKG_DLL_PREFIX}/poly2tri.dll"
          "${VCPKG_DLL_PREFIX}/kubazip.dll"
          "${VCPKG_DLL_PREFIX}/minizip.dll"
          "${VCPKG_DLL_PREFIX}/boost_serialization-vc${MSVC_TOOLSET_VERSION}-mt$<$<CONFIG:Debug>:-gd>-x64-${Boost_VERSION_MAJOR}_${Boost_VERSION_MINOR}.dll"
          # "${VCPKG_DLL_PREFIX}/boost_serialization-vc${MSVC_TOOLSET_VERSION}-mt-x64-${Boost_VERSION_MAJOR}_${Boost_VERSION_MINOR}.dll"
          "${VCPKG_DLL_PREFIX}/boost_filesystem-vc${MSVC_TOOLSET_VERSION}-mt$<$<CONFIG:Debug>:-gd>-x64-${Boost_VERSION_MAJOR}_${Boost_VERSION_MINOR}.dll"
          # "${VCPKG_DLL_PREFIX}/boost_filesystem-vc${MSVC_TOOLSET_VERSION}-mt-x64-${Boost_VERSION_MAJOR}_${Boost_VERSION_MINOR}.dll"
        DESTINATION
          ${_INSTALLDLL_DESTINATION}
        OPTIONAL
      )
    endif(DEFINED VCPKG_DLL_PREFIX)
  endif(WIN32)
endmacro()

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