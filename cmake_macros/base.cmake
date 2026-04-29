# using ${TARGET_LIB_PATH} to find_package
macro(FIND_PKG_PATH pkg_name alias)
    string(TOLOWER "${alias}" alias_lower)
    string(TOUPPER "${alias}" alias_upper)

    set(TARGET_${alias_upper}_PATH "" CACHE PATH "${pkg_name} install path")
    if(EXISTS ${TARGET_${alias_upper}_PATH})
    	message(STATUS "Directory to search ${pkg_name} at ${TARGET_${alias_upper}_PATH}")
    	list(APPEND CMAKE_PREFIX_PATH ${TARGET_${alias_upper}_PATH})
    else()
    	message(STATUS "File/Directory at variable TARGET_${alias_upper}_PATH not exists! ${TARGET_${alias_upper}_PATH}")
    endif()
  if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
  endif()
    find_package(${pkg_name} REQUIRED)
endmacro(FIND_PKG_PATH)


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
        "${CMAKE_SOURCE_DIR}/third_party/clarabel/rust_wrapper/target/$<IF:$<CONFIG:Debug>,debug,release>/clarabel_c.dll"
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
          "${VCPKG_DLL_PREFIX}/TracyClient.dll"
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
