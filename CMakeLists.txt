# More modern CMake
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(sire VERSION 1.0.0 LANGUAGES CXX)

# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(Boost_LIB_PREFIX "lib")
set(Boost_ARCHITECTURE "-x64")

if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/sire/sire-1.0")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/sire/sire-1.0")
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

set(TARGET_HPP_FCL_PATH "" CACHE PATH "Hpp-fcl install path")
# find Hpp-fcl
if(EXISTS ${TARGET_HPP_FCL_PATH})
	message(STATUS "Directory to search hpp-fcl at ${TARGET_HPP_FCL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_HPP_FCL_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_HPP_FCL_PATH not exists!")
endif()
find_package(hpp-fcl REQUIRED)

# Build lib simulator of sire
add_library(sire_simulator OBJECT)
target_sources(sire_simulator 
	PRIVATE 
		"src/simulator/dynamic_simulator.cpp"
	PUBLIC 
		"include/sire/simulator/dynamic_simulator.hpp"
	)

target_include_directories(sire_simulator
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)

target_compile_definitions(sire_simulator PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_simulator 
	PUBLIC 
		cxx_std_17
	)

install(DIRECTORY include/sire/simulator DESTINATION include/sire)

# Build lib collision of sire
add_library(sire_collision OBJECT)

target_compile_features(sire_collision 
	PRIVATE 
		cxx_std_17
	)

target_sources(sire_collision 
	PRIVATE 
		"src/collision/collision.cpp"
	PUBLIC 
		"include/sire/collision/collision.hpp"
	)

target_include_directories(sire_collision
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${ASSIMP_INCLUDE_DIRS}
		${hpp-fcl_INCLUDE_DIRS}
	)

target_compile_definitions(sire_collision PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_collision
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
	)

install(DIRECTORY include/sire/collision DESTINATION include/sire)

# Build library for sire
add_library(sire_lib 
	SHARED 
		$<TARGET_OBJECTS:sire_collision>
		$<TARGET_OBJECTS:sire_simulator>
	)

include(GenerateExportHeader)
generate_export_header(sire_lib EXPORT_MACRO_NAME SIRE_API)

target_include_directories(sire_lib
	PUBLIC 
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}> # sire_lib_EXPORTS
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
	)

message(STATUS "hpp-fcl ${hpp-fcl_LIBRARIES}")
set(PUB_RELY_LINK_LIB 
		${hpp-fcl_LIBRARIES}
		${aris_LIBRARIES}
)

target_link_libraries(sire_lib 
	PUBLIC 
		${PUB_RELY_LINK_LIB}
	PRIVATE
	)

set(sire_LIBRARIES 
	optimized sire::sire_lib 
	debug sire::debug::sire_lib
	)

install(TARGETS sire_lib 
	EXPORT sire-targets-debug 
	CONFIGURATIONS Debug 
	DESTINATION "lib/debug"
	)

install(TARGETS sire_lib 
	EXPORT sire-targets-release 
	CONFIGURATIONS Release 
	DESTINATION "lib/release"
	)

## install exports ##
install(EXPORT sire-targets-debug 
	NAMESPACE sire::debug:: 
	FILE sire-targets-debug.cmake 
	DESTINATION "share/sire/cmake"
	)

install(EXPORT sire-targets-release 
	NAMESPACE sire:: 
	FILE sire-targets-release.cmake 
	DESTINATION "share/sire/cmake"
	)

install(
	FILES 
		include/sire.hpp 
		${PROJECT_BINARY_DIR}/sire_lib_export.h
	DESTINATION 
		include
	)

## install config ##
include(CMakePackageConfigHelpers)
set(sire_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(sire_LIBRARIES ${sire_LIBRARIES} ${PUB_RELY_LINK_LIB})
configure_package_config_file(config/sire-config.cmake.in 
	${PROJECT_BINARY_DIR}/sire-config.cmake 
	INSTALL_DESTINATION "share/sire/cmake"
	)

install(FILES ${PROJECT_BINARY_DIR}/sire-config.cmake 
	DESTINATION "share/sire/cmake")
