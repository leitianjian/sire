# More modern CMake
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(aris_sim VERSION 1.0.0 LANGUAGES CXX)

# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/aris_sim/aris_sim-1.0")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/aris_sim/aris_sim-1.0")
endif(WIN32)

# Make sure kaanh and aris can be found
if(UNIX)
	set(TARGET_ARIS_PATH /usr/aris CACHE PATH "Aris install path")
	set(TARGET_KAANH_PATH /usr/kaanh CACHE PATH "Kaanh install path")
endif(UNIX)
if(WIN32)
	set(TARGET_ARIS_PATH C:/aris CACHE PATH "Aris install path")
endif(WIN32)

# Find kaanh and aris
if(EXISTS ${TARGET_ARIS_PATH})
	message(STATUS "Directory to search aris at ${TARGET_ARIS_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_ARIS_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_ARIS_PATH not exists!")
endif()
find_package(aris REQUIRED)

if(EXISTS ${TARGET_KAANH_PATH})
	message(STATUS "Directory to search kaanh at ${TARGET_KAANH_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_KAANH_PATH})
else()
	message(WARNING "File/Directory at variable KAANH_INSTALL_PATH not exists!")
endif()
find_package(kaanh)

# set ccd and fcl path
# set(TARGET_CCD_PATH "" CACHE PATH "Ccd install path")
# set(TARGET_FCL_PATH "" CACHE PATH "Fcl install path")

# find ccd
# if(EXISTS ${TARGET_CCD_PATH})
# 	message(STATUS "Directory to search ccd at ${TARGET_CCD_PATH}")
# 	list(APPEND CMAKE_PREFIX_PATH ${TARGET_CCD_PATH})
# else()
# 	message(WARNING "File/Directory at variable TARGET_CCD_PATH not exists!")
# endif()
# find_package(ccd REQUIRED)
# message(STATUS ${CCD_INCLUDE_DIRS})
# 
# # find fcl
# if(EXISTS ${TARGET_FCL_PATH})
# 	message(STATUS "Directory to search fcl at ${TARGET_FCL_PATH}")
# 	list(APPEND CMAKE_PREFIX_PATH ${TARGET_FCL_PATH})
# else()
# 	message(WARNING "File/Directory at variable TARGET_FCL_PATH not exists!")
# endif()
# find_package(fcl REQUIRED)

set(TARGET_ASSIMP_PATH "" CACHE PATH "Assimp install path")
# find Assimp
if(EXISTS ${TARGET_ASSIMP_PATH})
	message(STATUS "Directory to search Assimp at ${TARGET_ASSIMP_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_ASSIMP_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_ASSIMP_PATH not exists!")
endif()
find_package(assimp REQUIRED)

set(TARGET_HPP_FCL_PATH "" CACHE PATH "Hpp-fcl install path")

# find Hpp-fcl
if(EXISTS ${TARGET_HPP_FCL_PATH})
	message(STATUS "Directory to search Assimp at ${TARGET_HPP_FCL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_HPP_FCL_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_HPP_FCL_PATH not exists!")
endif()
find_package(hpp-fcl REQUIRED)

# Build lib simulator of aris_sim
add_library(aris_sim_simulator OBJECT)
target_sources(aris_sim_simulator 
	PRIVATE 
		"src/simulator/dynamic_simulator.cpp"
	PUBLIC 
		"include/aris_sim/simulator/dynamic_simulator.hpp"
	)

target_include_directories(aris_sim_simulator
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # aris_sim_lib_EXPORTS
	)

target_compile_definitions(aris_sim_simulator PRIVATE aris_sim_lib_EXPORTS)
target_compile_features(aris_sim_simulator 
	PUBLIC 
		cxx_std_17
	)

install(DIRECTORY include/aris_sim/simulator DESTINATION include/aris_sim)

# Build lib collision of aris_sim
add_library(aris_sim_collision OBJECT)

target_compile_features(aris_sim_collision 
	PRIVATE 
		cxx_std_17
	)

target_sources(aris_sim_collision 
	PRIVATE 
		"src/collision/collision.cpp"
	PUBLIC 
		"include/aris_sim/collision/collision.hpp"
	)

target_include_directories(aris_sim_collision
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # aris_sim_lib_EXPORTS
		${ASSIMP_INCLUDE_DIRS}
		${hpp-fcl_INCLUDE_DIRS}
	)

target_compile_definitions(aris_sim_collision PRIVATE aris_sim_lib_EXPORTS)
target_compile_options(aris_sim_collision
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
	)

install(DIRECTORY include/aris_sim/collision DESTINATION include/aris_sim)

# Build library for aris_sim
add_library(aris_sim_lib 
	SHARED 
		$<TARGET_OBJECTS:aris_sim_collision>
		$<TARGET_OBJECTS:aris_sim_simulator>
	)

include(GenerateExportHeader)
generate_export_header(aris_sim_lib EXPORT_MACRO_NAME ARIS_SIM_API)

target_include_directories(aris_sim_lib
	PUBLIC 
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}> # aris_sim_lib_EXPORTS
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
	)

set(PUB_RELY_LINK_LIB 
		assimp::assimp 
		${hpp-fcl_LIBRARIES})

target_link_libraries(aris_sim_lib 
	PUBLIC 
		${PUB_RELY_LINK_LIB}
	PRIVATE
		${aris_LIBRARIES} 
	)

set(aris_sim_LIBRARIES 
	optimized aris_sim::aris_sim_lib 
	debug aris_sim::debug::aris_sim_lib
	)

install(TARGETS aris_sim_lib 
	EXPORT aris_sim-targets-debug 
	CONFIGURATIONS Debug 
	DESTINATION "lib/debug"
	)

install(TARGETS aris_sim_lib 
	EXPORT aris_sim-targets-release 
	CONFIGURATIONS Release 
	DESTINATION "lib/release"
	)

## install exports ##
install(EXPORT aris_sim-targets-debug 
	NAMESPACE aris_sim::debug:: 
	FILE aris_sim-targets-debug.cmake 
	DESTINATION "share/aris_sim/cmake"
	)

install(EXPORT aris_sim-targets-release 
	NAMESPACE aris_sim:: 
	FILE aris_sim-targets-release.cmake 
	DESTINATION "share/aris_sim/cmake"
	)

install(
	FILES 
		include/aris_sim.hpp 
		${PROJECT_BINARY_DIR}/aris_sim_lib_export.h
	DESTINATION 
		include
	)

## install config ##
include(CMakePackageConfigHelpers)
set(aris_sim_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(aris_sim_LIBRARIES ${aris_sim_LIBRARIES} ${PUB_RELY_LINK_LIB})
configure_package_config_file(config/aris_sim-config.cmake.in 
	${PROJECT_BINARY_DIR}/aris_sim-config.cmake 
	INSTALL_DESTINATION "share/aris_sim/cmake"
	)

install(FILES ${PROJECT_BINARY_DIR}/aris_sim-config.cmake 
	DESTINATION "share/aris_sim/cmake")
