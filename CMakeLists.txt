# More modern CMake
cmake_minimum_required(VERSION 3.18.0)
project(sire VERSION 1.0.0)

# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/sire/sire-1.0")
endif(UNIX)

if(UNIX)
	add_definitions(-DUNIX)
	add_definitions(-fPIC)
endif(UNIX)

if(WIN32 AND NOT DEFINED CMAKE_INSTALL_PREFIX)
	set(CMAKE_INSTALL_PREFIX "C:/sire/sire-1.0")
endif(WIN32 AND NOT DEFINED CMAKE_INSTALL_PREFIX)

if(WIN32)
	add_definitions(-D_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING -D_CRT_SECURE_NO_WARNINGS)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251 /wd4819")
	add_definitions(${CMAKE_CXX_FLAGS})
endif(WIN32)

include(cmake_macros/base.cmake)

find_pkg_path(aris aris)

find_pkg_path(assimp assimp)

find_pkg_path(coal hpp_fcl)

find_pkg_path(stduuid stduuid)

find_pkg_path(msgpack-cxx msgpack_cxx)

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
		#"src/core/init_trigger.cpp"
	"src/core/nice_type_name.cpp"
		"src/core/timer.cpp"
		"src/core/sire_fixed_joint.cpp"
		"src/core/geometry/shape_base.cpp"
		"src/core/geometry/shape_calculator.cpp"
		"src/core/geometry/box_shape.cpp"
		"src/core/geometry/box_geometry.cpp"
		"src/core/geometry/capsule_shape.cpp"
		"src/core/geometry/capsule_geometry.cpp"
		"src/core/geometry/cylinder_shape.cpp"
		"src/core/geometry/cylinder_geometry.cpp"
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
		#"include/sire/core/init_trigger.hpp"
		"include/sire/core/timer.hpp"
		"include/sire/core/material_pair_prop.hpp"
		"include/sire/core/material_manager.hpp"
		"include/sire/core/nice_type_name.hpp"
		"include/sire/core/sire_fixed_joint.hpp"
		"include/sire/core/geometry/shape_base.hpp"
		"include/sire/core/geometry/shape_calculator.hpp"
		"include/sire/core/geometry/sphere_shape.hpp"
		"include/sire/core/geometry/sphere_geometry.hpp"
		"include/sire/core/geometry/box_shape.hpp"
		"include/sire/core/geometry/box_geometry.hpp"
		"include/sire/core/geometry/capsule_shape.hpp"
		"include/sire/core/geometry/capsule_geometry.hpp"
		"include/sire/core/geometry/cylinder_shape.hpp"
		"include/sire/core/geometry/cylinder_geometry.hpp"
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

add_subdirectory(src/log)

add_library(sire_actuator OBJECT)
target_compile_definitions(sire_actuator PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_actuator PUBLIC cxx_std_17)
target_sources(sire_actuator
	PRIVATE
		"src/actuator/actuator.cpp"
	PUBLIC
		"include/sire/actuator/actuator.hpp"
)
target_include_directories(sire_actuator
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
install(DIRECTORY include/sire/actuator DESTINATION include/sire)

# TODO(ltj)  Physics cmakelist to sub-directory
# Build lib physics of sire
add_library(sire_physics OBJECT)
target_compile_features(sire_physics PRIVATE cxx_std_17)
target_compile_definitions(sire_physics PRIVATE sire_lib_EXPORTS)
target_compile_options(sire_physics
	PRIVATE
		$<$<CXX_COMPILER_ID:MSVC>:/bigobj>
		$<$<CXX_COMPILER_ID:MSVC>:/Zc:preprocessor>
	# $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
)
target_sources(sire_physics
	PRIVATE
		"src/physics/geometry/collidable.cpp"
		"src/physics/geometry/collidable_geometry.cpp"
		"src/physics/geometry/sphere_collision_geometry.cpp"
		"src/physics/geometry/box_collision_geometry.cpp"
		"src/physics/geometry/capsule_collision_geometry.cpp"
		"src/physics/geometry/cylinder_collision_geometry.cpp"
		"src/physics/geometry/mesh_collision_geometry.cpp"
		"src/physics/contact/contact_solver.cpp"
		"src/physics/contact/stiffness_damping_contact_solver.cpp"
		"src/physics/contact/continuous_force_contact_solver.cpp"
		"src/physics/contact/avg_force_contact_solver.cpp"
		"src/physics/contact/contact_position_force_solver.cpp"
		"src/physics/collision/collision_detection.cpp"
		"src/physics/collision/collision_filter.cpp"
		"src/physics/collision/collision_exists_callback.cpp"
		"src/physics/collision/collided_objects_callback.cpp"
		"src/physics/collision/penetration_as_point_pair_callback.cpp"
		"src/physics/common/point_pair_contact_info.cpp"
		"src/physics/common/motion_force.cpp"
		"src/physics/physics_engine.cpp"
		"src/physics/utils.cpp"
	PUBLIC
		"include/sire/physics/geometry/collidable.hpp"
		"include/sire/physics/geometry/collidable_geometry.hpp"
		"include/sire/physics/geometry/sphere_collision_geometry.hpp"
		"include/sire/physics/geometry/box_collision_geometry.hpp"
		"include/sire/physics/geometry/capsule_collision_geometry.hpp"
		"include/sire/physics/geometry/cylinder_collision_geometry.hpp"
		"include/sire/physics/geometry/mesh_collision_geometry.hpp"
		"include/sire/physics/contact/contact_solver.hpp"
		"include/sire/physics/contact/stiffness_damping_contact_solver.hpp"
		"include/sire/physics/contact/continuous_force_contact_solver.hpp"
		"include/sire/physics/contact/avg_force_contact_solver.hpp"
		"include/sire/physics/contact/contact_position_force_solver.hpp"
		"include/sire/physics/contact/contact_solver_result.hpp"
		"include/sire/physics/collision/collision_detection.hpp"
		"include/sire/physics/collision/collision_filter.hpp"
		"include/sire/physics/collision/collision_exists_callback.hpp"
		"include/sire/physics/collision/collided_objects_callback.hpp"
		"include/sire/physics/collision/penetration_as_point_pair_callback.hpp"
		"include/sire/physics/common/penetration_as_point_pair.hpp"
		"include/sire/physics/common/point_pair_contact_info.hpp"
		"include/sire/physics/common/motion_force.hpp"
		"include/sire/physics/physics_engine.hpp"
		"include/sire/physics/physics.hpp"
		"include/sire/physics/utils.hpp"
)
target_include_directories(sire_physics
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${coal_INCLUDE_DIRS}
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
		"src/integrator/semi_implicit_euler_integrator.cpp"
		"src/integrator/runge_kutta4_integrator.cpp"
	PUBLIC
		"include/sire/integrator/integrator_base.hpp"
		"include/sire/integrator/explicit_euler_integrator.hpp"
		"include/sire/integrator/semi_implicit_euler_integrator.hpp"
		"include/sire/integrator/runge_kutta4_integrator.hpp"
)

target_include_directories(sire_integrator
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${coal_INCLUDE_DIRS}
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
	# $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
)
target_compile_features(sire_simulator PUBLIC cxx_std_17)
target_sources(sire_simulator
	PRIVATE
		"src/simulator/recorder.cpp"
		"src/simulator/simulation_loop.cpp"
		"src/simulator/events.cpp"
		"src/simulator/simulation_result.cpp"
		"src/simulator/simulator_modules.cpp"
		# "src/simulator/event_generator.cpp"
		"src/simulator/event_manager.cpp"
		"src/simulator/controller.cpp"
		"src/simulator/joint_constraint_solver.cpp"
	PUBLIC
	"include/sire/simulator/recorder.hpp"
		"include/sire/simulator/simulation_loop.hpp"
		"include/sire/simulator/events.hpp"
		"include/sire/simulator/simulation_result.hpp"
		"include/sire/simulator/simulator_modules.hpp"
	# "include/sire/simulator/event_generator.hpp"
		"include/sire/simulator/event_manager.hpp"
		"include/sire/simulator/controller.hpp"
		"include/sire/simulator/joint_constraint_solver.hpp"
)

target_include_directories(sire_simulator
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${coal_INCLUDE_DIRS}
		${ASSIMP_INCLUDE_DIRS}
)
install(DIRECTORY include/sire/simulator DESTINATION include/sire)

# add_subdirectory(third_party/stduuid)
add_library(sire_server OBJECT)
target_compile_definitions(sire_server PRIVATE sire_lib_EXPORTS)
target_compile_features(sire_server PUBLIC cxx_std_17)
target_sources(sire_server
	PRIVATE
		"src/server/api.cpp"
		"src/server/interface.cpp"
	# "src/server/meshcat.cpp"
	# "src/plan/get_force_sensor_data_cmd.cpp"
		"src/plan/get_cmd.cpp"
		"src/plan/sire_get.cpp"
		"src/plan/sim_play.cpp"
		"src/plan/sim_reset.cpp"
		"src/plan/sire_get_debug.cpp"
		"src/plan/display3d_init_cmd.cpp"
		"src/plan/fce_control_test_cmd.cpp"
		"src/plan/execute_real_time_test_cmd.cpp"
		"src/plan/mvj_cmd.cpp"
		"src/plan/record_get.cpp"
		"src/server/rgba.cpp"
		"src/server/mongoose.c"
		"src/server/mongoose.h"
		"src/server/md5.cpp"
		"src/server/md5.h"
		"src/server/sha1.cpp"
		"src/server/sha1.h"
		"src/server/base64.cpp"
		"src/server/base64.hpp"
	# "src/server/sire_socket_server.cpp"
	# "src/server/uuid.h"
	"src/ext/tinyxml2.cpp"
	PUBLIC
		"include/sire/server/api.hpp"
		"include/sire/server/interface.hpp"
	# "include/sire/server/sire_socket_server.hpp"
	# "include/sire/server/meshcat_types.hpp"
	# "include/sire/server/meshcat.hpp"
	# "include/sire/plan/get_force_sensor_data_cmd.hpp"
		"include/sire/plan/get_cmd.hpp"
		"include/sire/plan/sire_get.hpp"
		"include/sire/plan/sire_get_debug.hpp"
		"include/sire/plan/sim_play.hpp"
		"include/sire/plan/sim_reset.hpp"
		"include/sire/plan/display3d_init_cmd.hpp"
		"include/sire/plan/fce_control_test_cmd.hpp"
		"include/sire/plan/execute_real_time_test_cmd.hpp"
		"include/sire/plan/mvj_cmd.hpp"
		"include/sire/plan/record_get.hpp"
		"include/sire/server/rgba.hpp"
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
		${coal_INCLUDE_DIRS}
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
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${coal_INCLUDE_DIRS}
		${ASSIMP_INCLUDE_DIRS}
)
install(DIRECTORY include/sire/middleware DESTINATION include/sire)

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

# Build library for sire
add_library(sire_lib
	SHARED
		$<TARGET_OBJECTS:sire_core>
		$<TARGET_OBJECTS:sire_log>
		$<TARGET_OBJECTS:sire_middleware>
		$<TARGET_OBJECTS:sire_physics>
		$<TARGET_OBJECTS:sire_integrator>
		$<TARGET_OBJECTS:sire_simulator>
		$<TARGET_OBJECTS:sire_server>
		$<TARGET_OBJECTS:sire_sensor>
		$<TARGET_OBJECTS:sire_actuator>
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
set(PUB_RELY_LINK_LIB ${coal_LIBRARIES} ${aris_LIBRARIES} msgpack-cxx)
message(STATUS ${coal_LIBRARIES})
target_link_libraries(sire_lib PUBLIC ${PUB_RELY_LINK_LIB})

if(BUILD_PYTHON)
	add_subdirectory(python/sire)
endif()

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

# install config #
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
# option(BUILD_DEMO "build sire demo" OFF)
message(STATUS "build demos ${BUILD_DEMO}")

if(BUILD_DEMO)
	add_subdirectory(demo)
endif(BUILD_DEMO)

# ---------------------- build tests ---------------------------
option(BUILD_TEST "build sire tests" OFF)

if(BUILD_TEST)
	enable_testing()
	add_subdirectory(test)
endif(BUILD_TEST)
