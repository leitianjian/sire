# More modern CMake
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(sire VERSION 1.0.0)

# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

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

# find Assimp
set(TARGET_ASSIMP_PATH "" CACHE PATH "Assimp install path")
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
	message(STATUS "Directory to search hpp-fcl at ${TARGET_HPP_FCL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_HPP_FCL_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_HPP_FCL_PATH not exists!")
endif()
find_package(hpp-fcl REQUIRED)

include(GenerateExportHeader)
# Build lib simulator of sire
add_library(sire_simulator OBJECT)
target_compile_definitions(sire_simulator PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_simulator PUBLIC cxx_std_17)
target_sources(sire_simulator 
	PRIVATE 
		"src/simulator/dynamic_simulator.cpp"
		"src/simulator/integrator.cpp"
	PUBLIC 
		"include/sire/simulator/simulator.hpp"
		"include/sire/simulator/dynamic_simulator.hpp"
		"include/sire/simulator/integrator.hpp"
	)

target_include_directories(sire_simulator
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/simulator DESTINATION include/sire)

# Build lib collision of sire
add_library(sire_collision OBJECT)
target_compile_features(sire_collision PRIVATE cxx_std_17)
target_compile_definitions(sire_collision PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_collision
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
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
install(DIRECTORY include/sire/collision DESTINATION include/sire)

add_library(sire_server OBJECT)
target_compile_definitions(sire_server PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_server PUBLIC cxx_std_17)
target_sources(sire_server 
	PRIVATE 
		"src/server/api.cpp"
		"src/server/interface.cpp"
		"src/server/middle_ware.cpp"
		"src/server/command.cpp"
		"src/server/mongoose.c"
		"src/server/mongoose.h"
		"src/server/md5.cpp"
		"src/server/md5.h"
		"src/ext/tinyxml2.cpp"
	PUBLIC 
		"include/sire/server/api.hpp"
		"include/sire/server/interface.hpp"
		"include/sire/server/middle_ware.hpp"
		"include/sire/server/command.hpp"
		"include/sire/ext/tinyxml2.h"
	)
target_include_directories(sire_server
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/server DESTINATION include/sire)

# add_library(sire_sensor OBJECT)
# target_compile_definitions(sire_sensor PRIVATE sire_lib_EXPORTS)
# target_compile_features(sire_sensor PUBLIC cxx_std_17)
# target_sources(sire_sensor 
# 	PRIVATE 
# 		"src/sensor/sensor.cpp"
# 	PUBLIC 
# 		"include/sire/sensor/sensor.hpp"
# 	)
# target_include_directories(sire_sensor
# 	PUBLIC
# 		$<INSTALL_INTERFACE:include> 
# 		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 		${aris_INCLUDE_DIRS}
# 	PRIVATE
# 		${PROJECT_SOURCE_DIR}/src
# 		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
# 	)
# install(DIRECTORY include/sire/sensor DESTINATION include/sire)

add_library(sire_controller OBJECT)
target_compile_definitions(sire_controller PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_controller PUBLIC cxx_std_17)
target_sources(sire_controller 
	PRIVATE 
		"src/controller/controller_sensor.cpp"
	PUBLIC 
		"include/sire/controller/controller_sensor.hpp"
		"include/sire/controller/controller.hpp"
	)
target_include_directories(sire_controller
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/controller DESTINATION include/sire)

add_library(sire_transfer OBJECT)
target_compile_definitions(sire_transfer PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_transfer PUBLIC cxx_std_17)
target_sources(sire_transfer 
	PRIVATE 
		"src/transfer/sire_transfer.cpp"
	PUBLIC 
		"include/sire/transfer/sire_transfer.hpp"
		"include/sire/transfer/transfer.hpp"
	)
target_include_directories(sire_transfer
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/transfer DESTINATION include/sire)

# Build library for sire
add_library(sire_lib 
	SHARED 
		$<TARGET_OBJECTS:sire_collision>
		$<TARGET_OBJECTS:sire_simulator>
		$<TARGET_OBJECTS:sire_server>
		$<TARGET_OBJECTS:sire_controller>
		$<TARGET_OBJECTS:sire_transfer>
	)
generate_export_header(sire_lib EXPORT_MACRO_NAME SIRE_API)
target_compile_features(sire_lib PUBLIC cxx_std_17)
target_include_directories(sire_lib
	PUBLIC 
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}> # sire_lib_EXPORTS
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
	)
set(PUB_RELY_LINK_LIB ${hpp-fcl_LIBRARIES} ${aris_LIBRARIES})
target_link_libraries(sire_lib PUBLIC ${PUB_RELY_LINK_LIB})

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

# ---------------------- build demos ---------------------------
option(BUILD_DEMOS "build sire demos" OFF)
if(BUILD_DEMOS)
	set(SIRE_DEMOS demo_server_controller_sensor)
	foreach(sire_demo ${SIRE_DEMOS})
		file(GLOB SOURCES demo/${sire_demo}/*.h demo/${sire_demo}/*.cpp)
		add_executable(${sire_demo} ${SOURCES})
		target_include_directories(${sire_demo} PRIVATE ${aris_INCLUDE_DIRS} ${PROJECT_BINARY_DIR})
		target_link_libraries(${sire_demo} sire_server sire_controller sire_transfer sire_simulator ${aris_LIBRARIES})
		target_compile_features(${sire_demo} PUBLIC cxx_std_17)
		install(TARGETS ${sire_demo} DESTINATION demo/${sire_demo})
		install(TARGETS sire_lib DESTINATION demo/${sire_demo})
		install(FILES "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll" DESTINATION demo/${sire_demo})
		if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${sire_demo}/config)
			install(DIRECTORY demo/${sire_demo}/config/ DESTINATION demo/${sire_demo})
		endif()
		if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${sire_demo}/resource)
			install(DIRECTORY demo/${sire_demo}/resource/ DESTINATION resource/${sire_demo})
		endif()
	endforeach(sire_demo)
endif(BUILD_DEMOS)