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
# if(UNIX)
# 	set(TARGET_ARIS_PATH /usr/aris CACHE PATH "Aris install path")
# 	set(TARGET_KAANH_PATH /usr/kaanh CACHE PATH "Kaanh install path")
# endif(UNIX)
# if(WIN32)
# 	set(TARGET_ARIS_PATH C:/aris CACHE PATH "Aris install path")
# endif(WIN32)

# Find kaanh and aris
if(EXISTS ${TARGET_ARIS_PATH})
	message(STATUS "Directory to search aris at ${TARGET_ARIS_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_ARIS_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_ARIS_PATH not exists!")
endif()
find_package(aris REQUIRED)

# if(EXISTS ${TARGET_KAANH_PATH})
# 	message(STATUS "Directory to search kaanh at ${TARGET_KAANH_PATH}")
# 	list(APPEND CMAKE_PREFIX_PATH ${TARGET_KAANH_PATH})
# else()
# 	message(WARNING "File/Directory at variable KAANH_INSTALL_PATH not exists!")
# endif()
# find_package(kaanh)

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
add_library(sire_core OBJECT)
target_compile_definitions(sire_core PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_core PUBLIC cxx_std_17)
target_sources(sire_core 
	PRIVATE 
		"src/core/module_base.cpp"
		"src/core/sire_assert_and_throw.cpp"
		"src/core/geometry/sphere_shape.cpp"
		"src/core/geometry/box_shape.cpp"
		"src/core/geometry/sphere_geometry.cpp"
		"src/core/geometry/box_geometry.cpp"
		"src/core/geometry/shape_base.cpp"
		"src/core/geometry/geometry_base.cpp"
		"src/core/geometry/geometry_on_part.cpp"
	PUBLIC 
		"include/sire/core/module_base.hpp"
		"include/sire/core/constants.hpp"
		"include/sire/core/sire_assert.hpp"
		"include/sire/core/sire_throw.hpp"
		"include/sire/core/sire_assertion_error.hpp"
		"include/sire/core/sire_log.hpp"
		"include/sire/core/sire_decl_def_macro.hpp"
		"include/sire/core/geometry/sphere_shape.hpp"
		"include/sire/core/geometry/box_shape.hpp"
		"include/sire/core/geometry/shape_base.hpp"
		"include/sire/core/geometry/sphere_geometry.hpp"
		"include/sire/core/geometry/box_geometry.hpp"
		"include/sire/core/geometry/geometry_base.hpp"
		"include/sire/core/geometry/geometry_on_part.hpp"
	)
target_include_directories(sire_core
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/core DESTINATION include/sire)

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
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/simulator DESTINATION include/sire)

# TODO(ltj)  Physics cmakelist to sub-directory
# Build lib physics of sire
add_library(sire_physics OBJECT)
target_compile_features(sire_physics PRIVATE cxx_std_17)
target_compile_definitions(sire_physics PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_physics
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
	)
target_sources(sire_physics 
	PRIVATE 
		"src/physics/geometry/collidable.cpp"
		"src/physics/geometry/collidable_geometry.cpp"
		"src/physics/geometry/sphere_collision_geometry.cpp"
		"src/physics/geometry/box_collision_geometry.cpp"
		"src/physics/geometry/mesh_collision_geometry.cpp"
		"src/physics/collision/collision.cpp"
		"src/physics/collision/collision_detection_engine.cpp"
		"src/physics/collision/collision_module.cpp"
		"src/physics/collision/collision_filter.cpp"
		"src/physics/collision/collision_exists_callback.cpp"
		"src/physics/collision/collided_objects_callback.cpp"
		"src/physics/contact/contact_engine.cpp"
		"src/physics/physics_engine.cpp"

		"src/physics/contact/test/ball_rebound.cpp"
	PUBLIC 
		"include/sire/physics/geometry/collidable.hpp"
		"include/sire/physics/geometry/collidable_geometry.hpp"
		"include/sire/physics/geometry/sphere_collision_geometry.hpp"
		"include/sire/physics/geometry/box_collision_geometry.hpp"
		"include/sire/physics/geometry/mesh_collision_geometry.hpp"
		"include/sire/physics/collision/collision.hpp"
		"include/sire/physics/collision/collision_detection_engine.hpp"
		"include/sire/physics/collision/collision_module.hpp"
		"include/sire/physics/collision/collision_filter.hpp"
		"include/sire/physics/collision/collision_exists_callback.hpp"
		"include/sire/physics/collision/collided_objects_callback.hpp"
		"include/sire/physics/contact/contact_engine.hpp"
		"include/sire/physics/physics_engine.hpp"

		"include/sire/physics/contact/test/ball_rebound.hpp"
	)
target_include_directories(sire_physics
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${hpp-fcl_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${ASSIMP_INCLUDE_DIRS}
	)
install(DIRECTORY include/sire/physics DESTINATION include/sire)

add_library(sire_server OBJECT)
target_compile_definitions(sire_server PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_server PUBLIC cxx_std_17)
target_sources(sire_server 
	PRIVATE 
		"src/server/api.cpp"
		"src/server/interface.cpp"
		"src/plan/get_force_sensor_data_cmd.cpp"
		"src/plan/get_cmd.cpp"
		"src/plan/get_geometry_cmd.cpp"
		"src/plan/fce_control_test_cmd.cpp"
		"src/plan/execute_real_time_test_cmd.cpp"
		"src/plan/mvj_cmd.cpp"
		"src/server/mongoose.c"
		"src/server/mongoose.h"
		"src/server/md5.cpp"
		"src/server/md5.h"
		"src/ext/tinyxml2.cpp"
	PUBLIC 
		"include/sire/server/api.hpp"
		"include/sire/server/interface.hpp"
		"include/sire/plan/get_force_sensor_data_cmd.hpp"
		"include/sire/plan/get_cmd.hpp"
		"include/sire/plan/get_geometry_cmd.hpp"
		"include/sire/plan/fce_control_test_cmd.hpp"
		"include/sire/plan/execute_real_time_test_cmd.hpp"
		"include/sire/plan/mvj_cmd.hpp"
		"include/sire/ext/tinyxml2.h"
		"include/sire/ext/json.hpp"
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

add_library(sire_middleware OBJECT)
target_compile_definitions(sire_middleware PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_middleware PUBLIC cxx_std_17)
target_sources(sire_middleware 
	PRIVATE 
		"src/middleware/program_middleware.cpp"
		"src/middleware/sire_middleware.cpp"
		"src/middleware/sire_program_middleware.cpp"
	PUBLIC 
		"include/sire/middleware/program_middleware.hpp"
		"include/sire/middleware/sire_middleware.hpp"
		"include/sire/middleware/sire_program_middleware.hpp"
		"include/sire/ext/json.hpp"
	)
target_include_directories(sire_middleware
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/middleware DESTINATION include/sire)

add_library(sire_controller OBJECT)
target_compile_definitions(sire_controller PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_controller PUBLIC cxx_std_17)
target_sources(sire_controller 
	PRIVATE 
		"src/controller/controller_sensor.cpp"
	PUBLIC 
		"include/sire/controller/controller_sensor.hpp"
		"include/sire/controller/controller.hpp"
		"include/sire/ext/json.hpp"
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
		"src/transfer/force_control_simulation_transfer.cpp"
		"src/transfer/part_pq_transfer.cpp"
		"src/transfer/virtual_force_sensor_transfer.cpp"
	PUBLIC 
		"include/sire/transfer/force_control_simulation_transfer.hpp"
		"include/sire/transfer/part_pq_transfer.hpp"
		"include/sire/transfer/virtual_force_sensor_transfer.hpp"
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

# Build lib CAM backend of sire
add_library(sire_cam_backend OBJECT)
target_compile_features(sire_cam_backend PRIVATE cxx_std_17)
target_compile_definitions(sire_cam_backend PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_cam_backend
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
	)
target_sources(sire_cam_backend 
	PRIVATE 
		"src/cam_backend/cam_backend.cpp"
		"src/cam_backend/command.cpp"
	PUBLIC 
		"include/sire/cam_backend/cam_backend.hpp"
		"include/sire/cam_backend/command.hpp"
	)
target_include_directories(sire_cam_backend
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${hpp-fcl_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${ASSIMP_INCLUDE_DIRS}
	)
install(DIRECTORY include/sire/cam_backend DESTINATION include/sire)

# Build library for sire
add_library(sire_lib 
	SHARED 
		$<TARGET_OBJECTS:sire_core>
		$<TARGET_OBJECTS:sire_middleware>
		$<TARGET_OBJECTS:sire_physics>
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
	add_subdirectory(demo)
#	set(SIRE_DEMOS demo_server_controller_sensor demo_server_integrator_dynamics demo_middleware_collision demo_duan_cam_backend)
#	foreach(sire_demo ${SIRE_DEMOS})
#		file(GLOB SOURCES demo/${sire_demo}/*.h demo/${sire_demo}/*.cpp)
#		add_executable(${sire_demo} ${SOURCES})
#		target_include_directories(${sire_demo} PRIVATE ${aris_INCLUDE_DIRS} ${PROJECT_BINARY_DIR} ${hpp-fcl_INCLUDE_DIRS})
#		target_link_libraries(${sire_demo} sire_core sire_server sire_middleware sire_controller sire_transfer sire_simulator sire_physics sire_cam_backend ${PUB_RELY_LINK_LIB})
#		target_compile_features(${sire_demo} PUBLIC cxx_std_17)
#		install(
#			TARGETS 
#				sire_lib 
#				${sire_demo} 
#			CONFIGURATIONS
#				Debug
#			DESTINATION 
#				demo/debug/${sire_demo}
#			)
#		install(
#			TARGETS 
#				sire_lib 
#				${sire_demo} 
#			CONFIGURATIONS
#				Release
#			DESTINATION 
#				demo/release/${sire_demo}
#			)
#		install(
#			FILES 
#				"${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll" 
#				"${TARGET_HPP_FCL_PATH}/bin/hpp-fcl.dll"
#				"${assimp_DIR}/../../debug/bin/assimp-vc${MSVC_TOOLSET_VERSION}-mtd.dll"
#				"${assimp_DIR}/../../debug/bin/zlibd1.dll"
#				"${assimp_DIR}/../../debug/bin/pugixml.dll"
#			CONFIGURATIONS
#				Debug
#			DESTINATION 
#				demo/debug/${sire_demo}
#			)
#		install(
#			FILES 
#				"${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll" 
#				"${TARGET_HPP_FCL_PATH}/bin/hpp-fcl.dll"
#				"${assimp_DIR}/../../bin/assimp-vc${MSVC_TOOLSET_VERSION}-mt.dll"
#				"${assimp_DIR}/../../bin/zlib1.dll"
#				"${assimp_DIR}/../../bin/pugixml.dll"
#			CONFIGURATIONS
#				Release
#			DESTINATION 
#				demo/release/${sire_demo}
#			)
#		if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${sire_demo}/config)
#			install(DIRECTORY config CONFIGURATIONS Release DESTINATION demo/release/${sire_demo})
#			install(DIRECTORY config CONFIGURATIONS Debug DESTINATION demo/debug/${sire_demo})
#		endif()
#		# if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${sire_demo}/resource)
#		# 	install(DIRECTORY resource DESTINATION demo/${sire_demo})
#		# 	install(DIRECTORY resource DESTINATION resource/${sire_demo})
#		# endif()
#	endforeach(sire_demo)
endif(BUILD_DEMOS)

# ---------------------- build tests ---------------------------
option(BUILD_TESTS "build sire tests" OFF)
if(BUILD_TESTS)
	add_subdirectory(tests)
endif(BUILD_TESTS)