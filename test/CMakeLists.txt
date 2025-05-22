if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
	cmake_policy(SET CMP0135 NEW)
endif()

include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(base_factory_test "core/base_factory_test.cpp")
target_compile_features(base_factory_test PUBLIC cxx_std_17)
target_include_directories(base_factory_test
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(base_factory_test
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

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

add_executable(material_pair_prop_test "core/material_pair_prop_test.cpp")
target_compile_features(material_pair_prop_test PUBLIC cxx_std_17)
target_include_directories(material_pair_prop_test
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(material_pair_prop_test
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

add_executable(material_manager_test "core/material_manager_test.cpp")
target_compile_features(material_manager_test PUBLIC cxx_std_17)
target_include_directories(material_manager_test
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(material_manager_test
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

# add_executable(model_data_copy_test "model/model_data_copy_test.cpp")
# target_compile_features(model_data_copy_test PUBLIC cxx_std_17)
# target_include_directories(model_data_copy_test
# 	PRIVATE
# 		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 		${aris_INCLUDE_DIRS}
# 		${coal_INCLUDE_DIRS}
# 		${ASSIMP_INCLUDE_DIRS}
# 		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
# )
# target_link_libraries(model_data_copy_test
# 	PRIVATE
# 		GTest::gtest_main
# 		Eigen3::Eigen
# 		sire_physics
# 		sire_log
# 		sire_core
# 		sire_simulator
# 		sire_middleware
# 		sire_server
# 		sire_integrator
# 		sire_actuator
# 		${PUB_RELY_LINK_LIB}
# )

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

add_executable(force_screw_test "core/force_screw_test.cpp")
target_compile_features(force_screw_test PUBLIC cxx_std_17)
target_include_directories(force_screw_test
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
		${aris_INCLUDE_DIRS}
)
target_link_libraries(force_screw_test
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(collision_detection_test
	"physics/collision/physics_engine_test.cpp")
target_compile_features(collision_detection_test PUBLIC cxx_std_17)
target_include_directories(collision_detection_test
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${coal_INCLUDE_DIRS}
		${ASSIMP_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(collision_detection_test
	PRIVATE
		GTest::gtest_main
		Eigen3::Eigen
		sire_physics
		sire_log
		sire_core
		sire_simulator
		sire_middleware
		sire_server
		sire_integrator
		sire_actuator
		${PUB_RELY_LINK_LIB}
)

add_executable(multi_contact_mass_matrix
	"physics/contact/multi_contact_mass_matrix.cpp")
target_compile_features(multi_contact_mass_matrix PUBLIC cxx_std_17)
target_include_directories(multi_contact_mass_matrix
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(multi_contact_mass_matrix
	PRIVATE
		GTest::gtest_main
		sire_core
		${PUB_RELY_LINK_LIB}
)

add_executable(geometry_serialization_test
	"physics/geometry/geometry_serialization_test.cpp")
target_compile_features(geometry_serialization_test PUBLIC cxx_std_17)
target_include_directories(geometry_serialization_test
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${aris_INCLUDE_DIRS}
		${coal_INCLUDE_DIRS}
		${ASSIMP_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
)
target_link_libraries(geometry_serialization_test
	PRIVATE
		GTest::gtest_main
		sire_physics
		sire_log
		sire_core
		sire_simulator
		sire_middleware
		sire_server
		sire_integrator
		sire_actuator
		${PUB_RELY_LINK_LIB}
)

if(WIN32)
	string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)

	if(DEFINED VCPKG_INSTALLED_DIR)
		if(CMAKE_BUILD_TYPE_LOWER STREQUAL "debug")
			set(VCPKG_DLL_PREFIX ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/bin)
		elseif(CMAKE_BUILD_TYPE_LOWER STREQUAL "release")
			set(VCPKG_DLL_PREFIX ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin)
		endif()
	endif(DEFINED VCPKG_INSTALLED_DIR)

	add_custom_command(TARGET collision_detection_test POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:collision_detection_test> $<TARGET_FILE_DIR:collision_detection_test>
		COMMAND_EXPAND_LISTS
	)

	add_custom_command(TARGET collision_detection_test POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy
		"${CMAKE_CURRENT_SOURCE_DIR}/model/a1_modified.xml"
		"${TARGET_ARIS_PATH}/lib/${CMAKE_BUILD_TYPE_LOWER}/aris_lib.dll"
		"${TARGET_HPP_FCL_PATH}/bin/coal.dll"
		"${VCPKG_DLL_PREFIX}/assimp-vc${MSVC_TOOLSET_VERSION}-mt$<$<CONFIG:Debug>:d>.dll"
		"${VCPKG_DLL_PREFIX}/zlib$<$<CONFIG:Debug>:d>1.dll"
		"${VCPKG_DLL_PREFIX}/pugixml.dll"
		"${VCPKG_DLL_PREFIX}/boost_serialization-vc${MSVC_TOOLSET_VERSION}-mt$<$<CONFIG:Debug>:-gd>-x64-${Boost_VERSION_MAJOR}_${Boost_VERSION_MINOR}.dll"
		"${VCPKG_DLL_PREFIX}/boost_filesystem-vc${MSVC_TOOLSET_VERSION}-mt$<$<CONFIG:Debug>:-gd>-x64-${Boost_VERSION_MAJOR}_${Boost_VERSION_MINOR}.dll"
		$<TARGET_FILE_DIR:collision_detection_test>
	)

	# add_custom_command(TARGET collision_detection_test POST_BUILD
	# COMMAND ${CMAKE_COMMAND} -E echo
	# "target runtime dll = $<TARGET_RUNTIME_DLLS:collision_detection_test>")
	add_custom_command(TARGET sorted_pair_test POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:sorted_pair_test> $<TARGET_FILE_DIR:sorted_pair_test>
		COMMAND_EXPAND_LISTS
	)

	# add_custom_command(TARGET sorted_pair_test POST_BUILD
	# COMMAND ${CMAKE_COMMAND} -E echo
	# "target runtime dll = $<TARGET_RUNTIME_DLLS:sorted_pair_test>")
endif(WIN32)

include(GoogleTest)

# set not to run test post build
# gtest_discover_tests(collision_detection_test sorted_pair_test multi_contact_mass_matrix force_screw_test)