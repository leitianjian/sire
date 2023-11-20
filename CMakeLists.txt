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
message(${hpp-fcl_INCLUDE_DIRS})

include(GenerateExportHeader)
add_library(sire_core OBJECT)
target_compile_definitions(sire_core PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_core PUBLIC cxx_std_17)
target_sources(sire_core 
	PRIVATE 
		"src/core/contact_pair_manager.cpp"
		"src/core/module_base.cpp"
		"src/core/sire_assert_and_throw.cpp"
		"src/core/force_screw.cpp"
		"src/core/prop_map.cpp"
		"src/core/material_manager.cpp"
		"src/core/material_pair_prop.cpp"
		"src/core/sorted_pair.cpp"
		"src/core/base_factory.cpp"
		"src/core/trigger_base.cpp"
		"src/core/event_base.cpp"
		"src/core/handler_base.cpp"
		"src/core/event_manager.cpp"
		#"src/core/init_trigger.cpp"
		"src/core/timer.cpp"
		"src/core/geometry/shape_base.cpp"
		"src/core/geometry/box_shape.cpp"
		"src/core/geometry/box_geometry.cpp"
		"src/core/geometry/mesh_shape.cpp"
		"src/core/geometry/mesh_geometry.cpp"
		"src/core/geometry/sphere_shape.cpp"
		"src/core/geometry/sphere_geometry.cpp"
		"src/core/geometry/geometry_base.cpp"
		"src/core/geometry/geometry_on_part.cpp"
		#"src/core/geometry/geometry_properties.cpp"
	PUBLIC 
		"include/sire/core/constants.hpp"
		"include/sire/core/contact_pair_manager.hpp"
		"include/sire/core/module_base.hpp"
		"include/sire/core/is_less_than_comparable.hpp"
		"include/sire/core/sire_assert.hpp"
		"include/sire/core/sire_throw.hpp"
		"include/sire/core/sire_assertion_error.hpp"
		"include/sire/core/sire_log.hpp"
		"include/sire/core/sire_decl_def_macro.hpp"
		"include/sire/core/sorted_pair.hpp"
		"include/sire/core/string_utils.hpp"
		"include/sire/core/force_screw.hpp"
		"include/sire/core/prop_map.hpp"
		"include/sire/core/base_factory.hpp"
		"include/sire/core/trigger_base.hpp"
		"include/sire/core/event_base.hpp"
		"include/sire/core/handler_base.hpp"
		"include/sire/core/event_manager.hpp"
		#"include/sire/core/init_trigger.hpp"
		"include/sire/core/timer.hpp"
		"include/sire/core/material_pair_prop.hpp"
		"include/sire/core/material_manager.hpp"
		"include/sire/core/geometry/shape_base.hpp"
		"include/sire/core/geometry/sphere_shape.hpp"
		"include/sire/core/geometry/sphere_geometry.hpp"
		"include/sire/core/geometry/box_shape.hpp"
		"include/sire/core/geometry/box_geometry.hpp"
		"include/sire/core/geometry/mesh_shape.hpp"
		"include/sire/core/geometry/mesh_geometry.hpp"
		"include/sire/core/geometry/geometry_base.hpp"
		"include/sire/core/geometry/geometry_on_part.hpp"
		#"include/sire/core/geometry/geometry_properties.hpp"
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

# TODO(ltj)  Physics cmakelist to sub-directory
# Build lib physics of sire
add_library(sire_physics OBJECT)
target_compile_features(sire_physics PRIVATE cxx_std_17)
target_compile_definitions(sire_physics PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_physics
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<CXX_COMPILER_ID:MSVC>:/Zc:preprocessor>
		$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
	)
target_sources(sire_physics 
	PRIVATE 
		"src/physics/geometry/collidable.cpp"
		"src/physics/geometry/collidable_geometry.cpp"
		"src/physics/geometry/sphere_collision_geometry.cpp"
		"src/physics/geometry/box_collision_geometry.cpp"
		"src/physics/geometry/mesh_collision_geometry.cpp"
		"src/physics/contact/contact_solver.cpp"
		"src/physics/contact/stiffness_damping_contact_solver.cpp"
		"src/physics/contact/continuous_force_contact_solver.cpp"
		"src/physics/collision/collision.cpp"
		"src/physics/collision/collision_detection.cpp"
		"src/physics/collision/collision_filter.cpp"
		"src/physics/collision/collision_exists_callback.cpp"
		"src/physics/collision/collided_objects_callback.cpp"
		"src/physics/collision/penetration_as_point_pair_callback.cpp"
		"src/physics/common/point_pair_contact_info.cpp"
		"src/physics/physics_engine.cpp"
		"src/physics/utils.cpp"
	PUBLIC 
		"include/sire/physics/geometry/collidable.hpp"
		"include/sire/physics/geometry/collidable_geometry.hpp"
		"include/sire/physics/geometry/sphere_collision_geometry.hpp"
		"include/sire/physics/geometry/box_collision_geometry.hpp"
		"include/sire/physics/geometry/mesh_collision_geometry.hpp"
		"include/sire/physics/contact/contact_solver.hpp"
		"include/sire/physics/contact/stiffness_damping_contact_solver.hpp"
		"include/sire/physics/contact/continuous_force_contact_solver.hpp"
		"include/sire/physics/contact/contact_solver_result.hpp"
		"include/sire/physics/collision/collision.hpp"
		"include/sire/physics/collision/collision_detection.hpp"
		"include/sire/physics/collision/collision_filter.hpp"
		"include/sire/physics/collision/collision_exists_callback.hpp"
		"include/sire/physics/collision/collided_objects_callback.hpp"
		"include/sire/physics/collision/penetration_as_point_pair_callback.hpp"
		"include/sire/physics/common/penetration_as_point_pair.hpp"
		"include/sire/physics/common/point_pair_contact_info.hpp"
		"include/sire/physics/physics_engine.hpp"
		"include/sire/physics/physics.hpp"
		"include/sire/physics/utils.hpp"
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

# Build lib integrator of sire
add_library(sire_integrator OBJECT)
target_compile_definitions(sire_integrator PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_integrator PUBLIC cxx_std_17)
target_sources(sire_integrator 
	PRIVATE 
		"src/integrator/integrator_base.cpp"
		"src/integrator/explicit_euler_integrator.cpp"
		"src/integrator/runge_kutta4_integrator.cpp"
	PUBLIC 
		"include/sire/integrator/integrator_base.hpp"
		"include/sire/integrator/explicit_euler_integrator.hpp"
		"include/sire/integrator/runge_kutta4_integrator.hpp"
	)

target_include_directories(sire_integrator
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
install(DIRECTORY include/sire/integrator DESTINATION include/sire)

# Build lib simulator of sire
add_library(sire_simulator OBJECT)
target_compile_definitions(sire_simulator PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_simulator
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<CXX_COMPILER_ID:MSVC>:/Zc:preprocessor>
		$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
	)
target_compile_features(sire_simulator PUBLIC cxx_std_17)
target_sources(sire_simulator 
	PRIVATE 
		"src/simulator/simulator.cpp"
		"src/simulator/simulator_base.cpp"
		#"src/simulator/event_manager.cpp"
		"src/simulator/events.cpp"
		"src/simulator/simple_event_manager.cpp"
		"src/simulator/dynamic_simulator.cpp"
		"src/simulator/simulator_modules.cpp"
	PUBLIC 
		"include/sire/simulator/simulator.hpp"
		"include/sire/simulator/simulator_base.hpp"
		#"include/sire/simulator/event_manager.hpp"
		"include/sire/simulator/events.hpp"
		"include/sire/simulator/simple_event_manager.hpp"
		"include/sire/simulator/dynamic_simulator.hpp"
		"include/sire/simulator/simulator_modules.hpp"
	)

target_include_directories(sire_simulator
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
install(DIRECTORY include/sire/simulator DESTINATION include/sire)

add_library(sire_server OBJECT)
target_compile_definitions(sire_server PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_server PUBLIC cxx_std_17)
target_sources(sire_server 
	PRIVATE 
		"src/server/api.cpp"
		"src/server/interface.cpp"
#		"src/plan/get_force_sensor_data_cmd.cpp"
		"src/plan/get_cmd.cpp"
		"src/plan/sire_get.cpp"
		"src/plan/display3d_init_cmd.cpp"
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
#		"include/sire/plan/get_force_sensor_data_cmd.hpp"
		"include/sire/plan/get_cmd.hpp"
		"include/sire/plan/sire_get.hpp"
		"include/sire/plan/display3d_init_cmd.hpp"
		"include/sire/plan/fce_control_test_cmd.hpp"
		"include/sire/plan/execute_real_time_test_cmd.hpp"
		"include/sire/plan/mvj_cmd.hpp"
		"include/sire/ext/tinyxml2.h"
		"include/sire/ext/json.hpp"
	)
option(INTERFACE_DEBUG "switch to debug interface" OFF)
target_include_directories(sire_server
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${ASSIMP_INCLUDE_DIRS}
		${hpp-fcl_INCLUDE_DIRS}
	)
install(DIRECTORY include/sire/server DESTINATION include/sire)
install(DIRECTORY include/sire/ext DESTINATION include/sire)
install(DIRECTORY include/sire/plan DESTINATION include/sire)

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
		${hpp-fcl_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${ASSIMP_INCLUDE_DIRS}
	)
install(DIRECTORY include/sire/middleware DESTINATION include/sire)

# add_library(sire_controller OBJECT)
# target_compile_definitions(sire_controller PRIVATE sire_lib_EXPORTS)
# target_compile_features(sire_controller PUBLIC cxx_std_17)
# target_sources(sire_controller 
# 	PRIVATE 
# 		"src/controller/controller_sensor.cpp"
# 	PUBLIC 
# 		"include/sire/controller/controller_sensor.hpp"
# 		"include/sire/controller/controller.hpp"
# 		"include/sire/ext/json.hpp"
# 	)
# target_include_directories(sire_controller
# 	PUBLIC
# 		$<INSTALL_INTERFACE:include> 
# 		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 		${aris_INCLUDE_DIRS}
# 	PRIVATE
# 		${PROJECT_SOURCE_DIR}/src
# 		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
# 	)
# install(DIRECTORY include/sire/controller DESTINATION include/sire)

add_library(sire_transfer OBJECT)
target_compile_definitions(sire_transfer PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_transfer PUBLIC cxx_std_17)
target_sources(sire_transfer 
	PRIVATE 
		"src/transfer/force_control_simulation_transfer.cpp"
		"src/transfer/simulation_transfer.cpp"
		"src/transfer/part_pq_transfer.cpp"
#		"src/transfer/virtual_force_sensor_transfer.cpp"
	PUBLIC 
		"include/sire/transfer/force_control_simulation_transfer.hpp"
		"include/sire/transfer/simulation_transfer.hpp"
		"include/sire/transfer/part_pq_transfer.hpp"
#		"include/sire/transfer/virtual_force_sensor_transfer.hpp"
	)
target_include_directories(sire_transfer
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
install(DIRECTORY include/sire/transfer DESTINATION include/sire)

add_library(sire_sensor OBJECT)
target_compile_definitions(sire_sensor PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_sensor PUBLIC cxx_std_17)
target_sources(sire_sensor 
	PRIVATE 
		"src/sensor/sensor.cpp"
	PUBLIC 
		"include/sire/sensor/sensor.hpp"
	)
target_include_directories(sire_sensor
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)
install(DIRECTORY include/sire/sensor DESTINATION include/sire)

# Build lib CAM backend of sire
add_library(sire_cam_backend OBJECT)
target_compile_features(sire_cam_backend PRIVATE cxx_std_17)
target_compile_definitions(sire_cam_backend PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_cam_backend
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<CXX_COMPILER_ID:MSVC>:/Zc:preprocessor>
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
		$<TARGET_OBJECTS:sire_integrator>
		$<TARGET_OBJECTS:sire_simulator>
		$<TARGET_OBJECTS:sire_server>
		$<TARGET_OBJECTS:sire_transfer>
		$<TARGET_OBJECTS:sire_sensor>
		$<TARGET_OBJECTS:sire_cam_backend>
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
endif(BUILD_DEMOS)

# ---------------------- build tests ---------------------------
option(BUILD_TESTS "build sire tests" OFF)
if(BUILD_TESTS)
	add_subdirectory(tests)
endif(BUILD_TESTS)