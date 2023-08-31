include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(sorted_pair_test "core/sorted_pair_test.cpp")
target_compile_features(sorted_pair_test PUBLIC cxx_std_17)
target_include_directories(sorted_pair_test
	PRIVATE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(sorted_pair_test
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

add_executable(string_utils_test "core/string_utils_test.cpp")
target_compile_features(string_utils_test PUBLIC cxx_std_17)
target_include_directories(string_utils_test
	PRIVATE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(string_utils_test
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

add_executable(prop_map_test "core/prop_map_test.cpp")
target_compile_features(prop_map_test PUBLIC cxx_std_17)
target_include_directories(prop_map_test
	PRIVATE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(prop_map_test
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

add_executable(model_data_copy_test "model/model_data_copy_test")
target_compile_features(model_data_copy_test PUBLIC cxx_std_17)
target_include_directories(model_data_copy_test
	PRIVATE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS} 
)

target_link_libraries(model_data_copy_test
	PRIVATE
		GTest::gtest_main
		${PUB_RELY_LINK_LIB}
)

add_executable(screw_test "model/screw_test.cpp")
target_compile_features(screw_test PUBLIC cxx_std_17)
target_include_directories(screw_test
	PRIVATE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS} 
)

target_link_libraries(screw_test
	PRIVATE
		GTest::gtest_main
		${PUB_RELY_LINK_LIB}
)

add_executable(collision_detection_test
	"physics/collision/physics_engine_test.cpp")
target_compile_features(collision_detection_test PUBLIC cxx_std_17)

target_include_directories(collision_detection_test
	PRIVATE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS} 
		${PROJECT_BINARY_DIR} 
		${hpp-fcl_INCLUDE_DIRS}
		${ASSIMP_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)

target_link_libraries(collision_detection_test
	PRIVATE
		GTest::gtest_main
		sire_physics
		sire_core
		sire_simulator
		sire_middleware
		sire_server
		sire_integrator
		sire_transfer
		${PUB_RELY_LINK_LIB}
)

add_executable(geometry_serialization_test
	"physics/geometry/geometry_serialization_test.cpp")
target_compile_features(geometry_serialization_test PUBLIC cxx_std_17)

target_include_directories(geometry_serialization_test
	PRIVATE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS} 
		${PROJECT_BINARY_DIR} 
		${hpp-fcl_INCLUDE_DIRS}
		${ASSIMP_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)

target_link_libraries(geometry_serialization_test
	PRIVATE
		GTest::gtest_main
		sire_physics
		sire_core
		sire_simulator
		sire_middleware
		sire_server
		sire_integrator
		sire_transfer
		${PUB_RELY_LINK_LIB}
)

add_custom_command(TARGET collision_detection_test POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:collision_detection_test> $<TARGET_RUNTIME_DLLS:collision_detection_test>
  COMMAND_EXPAND_LISTS
)
add_custom_command(TARGET collision_detection_test POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E echo 
  "target runtime dll = $<TARGET_RUNTIME_DLLS:collision_detection_test>")

add_custom_command(TARGET sorted_pair_test POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:sorted_pair_test> $<TARGET_RUNTIME_DLLS:sorted_pair_test>
  COMMAND_EXPAND_LISTS
)
add_custom_command(TARGET sorted_pair_test POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E echo 
  "target runtime dll = $<TARGET_RUNTIME_DLLS:sorted_pair_test>")

include(GoogleTest)
# set not to run test post build
gtest_discover_tests(collision_detection_test sorted_pair_test DISCOVERY_MODE PRE_TEST)