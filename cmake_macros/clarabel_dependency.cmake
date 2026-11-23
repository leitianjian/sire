# Prefer an already-built Clarabel C runtime for ordinary Sire development.
# A clean checkout falls back to Clarabel's source build. Set
# SIRE_CLARABEL_FORCE_SOURCE_BUILD=ON after changing Clarabel Rust sources.
option(SIRE_CLARABEL_FORCE_SOURCE_BUILD
       "Rebuild Clarabel from Rust sources instead of using its existing runtime" OFF)

set(_sire_clarabel_root "${CMAKE_SOURCE_DIR}/third_party/clarabel")
if(CMAKE_BUILD_TYPE MATCHES Release)
  set(_sire_clarabel_config release)
else()
  set(_sire_clarabel_config debug)
endif()
set(_sire_clarabel_dir
    "${_sire_clarabel_root}/rust_wrapper/target/${_sire_clarabel_config}")

if(WIN32)
  set(_sire_clarabel_link "${_sire_clarabel_dir}/clarabel_c.dll.lib")
  set(_sire_clarabel_runtime "${_sire_clarabel_dir}/clarabel_c.dll")
elseif(APPLE)
  set(_sire_clarabel_link "${_sire_clarabel_dir}/libclarabel_c.dylib")
  set(_sire_clarabel_runtime "${_sire_clarabel_link}")
else()
  set(_sire_clarabel_link "${_sire_clarabel_dir}/libclarabel_c.so")
  set(_sire_clarabel_runtime "${_sire_clarabel_link}")
endif()

if(SIRE_CLARABEL_FORCE_SOURCE_BUILD)
  add_subdirectory("${_sire_clarabel_root}" "${CMAKE_BINARY_DIR}/clarabel")
  set_property(TARGET libclarabel_c_shared PROPERTY
               SIRE_RUNTIME_FILE "${_sire_clarabel_runtime}")
  message(STATUS "Sire Clarabel dependency: forced upstream source build")
elseif(EXISTS "${_sire_clarabel_link}" AND EXISTS "${_sire_clarabel_runtime}")
  add_library(libclarabel_c_shared INTERFACE)
  target_link_libraries(libclarabel_c_shared INTERFACE "${_sire_clarabel_link}")
  target_include_directories(libclarabel_c_shared INTERFACE
                             "${_sire_clarabel_root}/include")
  set_property(TARGET libclarabel_c_shared PROPERTY
               SIRE_RUNTIME_FILE "${_sire_clarabel_runtime}")
  message(STATUS "Sire Clarabel dependency: existing ${_sire_clarabel_runtime}")
else()
  # The upstream custom target always runs cargo and `cargo install cbindgen`.
  # A tracked OUTPUT makes a clean build incremental and checked-in headers
  # mean cbindgen is unnecessary for ordinary Sire builds.
  find_program(_sire_cargo cargo REQUIRED)
  set(_sire_cargo_target "${CMAKE_BINARY_DIR}/clarabel-cargo")
  set(_sire_cargo_flags)
  if(CMAKE_BUILD_TYPE MATCHES Release)
    list(APPEND _sire_cargo_flags --release)
  endif()
  set(_sire_clarabel_dir "${_sire_cargo_target}/${_sire_clarabel_config}")
  if(WIN32)
    set(_sire_clarabel_link "${_sire_clarabel_dir}/clarabel_c.dll.lib")
    set(_sire_clarabel_runtime "${_sire_clarabel_dir}/clarabel_c.dll")
    set(_sire_clarabel_static "${_sire_clarabel_dir}/clarabel_c.lib")
  elseif(APPLE)
    set(_sire_clarabel_link "${_sire_clarabel_dir}/libclarabel_c.dylib")
    set(_sire_clarabel_runtime "${_sire_clarabel_link}")
    set(_sire_clarabel_static "${_sire_clarabel_dir}/libclarabel_c.a")
  else()
    set(_sire_clarabel_link "${_sire_clarabel_dir}/libclarabel_c.so")
    set(_sire_clarabel_runtime "${_sire_clarabel_link}")
    set(_sire_clarabel_static "${_sire_clarabel_dir}/libclarabel_c.a")
  endif()
  file(GLOB_RECURSE _sire_clarabel_sources CONFIGURE_DEPENDS
       "${_sire_clarabel_root}/rust_wrapper/src/*.rs"
       "${_sire_clarabel_root}/Clarabel.rs/src/*.rs")
  file(GLOB _sire_clarabel_manifests CONFIGURE_DEPENDS
       "${_sire_clarabel_root}/rust_wrapper/Cargo.*"
       "${_sire_clarabel_root}/rust_wrapper/build.rs"
       "${_sire_clarabel_root}/Clarabel.rs/Cargo.*"
       "${_sire_clarabel_root}/Clarabel.rs/build.rs")
  set(_sire_clarabel_byproducts "${_sire_clarabel_link}"
      "${_sire_clarabel_runtime}" "${_sire_clarabel_static}")
  list(REMOVE_DUPLICATES _sire_clarabel_byproducts)
  add_custom_command(
    OUTPUT "${CMAKE_BINARY_DIR}/clarabel-cargo-built.stamp"
    COMMAND ${CMAKE_COMMAND} -E env "CARGO_TARGET_DIR=${_sire_cargo_target}"
            "${_sire_cargo}" build ${_sire_cargo_flags}
    COMMAND ${CMAKE_COMMAND} -E touch
            "${CMAKE_BINARY_DIR}/clarabel-cargo-built.stamp"
    WORKING_DIRECTORY "${_sire_clarabel_root}/rust_wrapper"
    DEPENDS ${_sire_clarabel_sources} ${_sire_clarabel_manifests}
            "${CMAKE_CURRENT_LIST_FILE}"
    BYPRODUCTS ${_sire_clarabel_byproducts}
    VERBATIM)
  add_custom_target(sire_clarabel_c
                    DEPENDS "${CMAKE_BINARY_DIR}/clarabel-cargo-built.stamp")
  add_library(libclarabel_c_shared INTERFACE)
  target_link_libraries(libclarabel_c_shared INTERFACE "${_sire_clarabel_link}")
  target_include_directories(libclarabel_c_shared INTERFACE
                             "${_sire_clarabel_root}/include")
  add_dependencies(libclarabel_c_shared sire_clarabel_c)
  set_property(TARGET libclarabel_c_shared PROPERTY
               SIRE_RUNTIME_FILE "${_sire_clarabel_runtime}")
  message(STATUS "Sire Clarabel dependency: incremental source build")
endif()

unset(_sire_clarabel_root)
unset(_sire_clarabel_config)
unset(_sire_clarabel_dir)
unset(_sire_clarabel_link)
unset(_sire_clarabel_runtime)
unset(_sire_clarabel_static)
unset(_sire_cargo)
unset(_sire_cargo_target)
unset(_sire_cargo_flags)
unset(_sire_clarabel_sources)
unset(_sire_clarabel_manifests)
unset(_sire_clarabel_byproducts)
