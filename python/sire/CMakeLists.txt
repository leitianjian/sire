if(CMAKE_VERSION VERSION_GREATER "3.27.0")
  cmake_policy(SET CMP0148 NEW)
endif()

# if(EXISTS ${TARGET_PYTHON_PATH})
# message(STATUS "Directory to search Python at ${TARGET_PYTHON_PATH}")

# if(WIN32)
# set(Python3_ROOT_DIR ${TARGET_PYTHON_PATH})
# set(Python_EXECUTABLE "${TARGET_PYTHON_PATH}/python.exe")
# set(Python_LIBRARY "${TARGET_PYTHON_PATH}/libs/python313.lib")
# set(Python_INCLUDE_DIR "${TARGET_PYTHON_PATH}/Include")
# endif()
# else()
# message(STATUS "File/Directory at variable TARGET_PYTHON_PATH not exists! ${TARGET_PYTHON_PATH}")
# endif()

# message(STATUS "Find Python at ${Python3_ROOT_DIR}")
# find_package(Python COMPONENTS Development.Module REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Threads REQUIRED)
set(PYTHON_MODULE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/python_modules")
file(MAKE_DIRECTORY ${PYTHON_MODULE_OUTPUT_DIR})

# Clarabel doesn't export a CMake config, so we include it directly from the source tree
if(NOT TARGET libclarabel_c_shared)
	add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/clarabel ${CMAKE_BINARY_DIR}/clarabel)
endif()
# SIRE PYTHON BINDING
pybind11_add_module(sire
  "pybind11_json.hpp"
  "sire_bindings.cpp"
  "bindings_core.cpp"
  "bindings_model.cpp"
  "bindings_physics.cpp"
  "bindings_pointer_arrays.cpp"
  "bindings_rl.cpp"
  "bindings_server.cpp"
  "bindings_simulator.cpp"
  "bindings_utils.cpp")
target_compile_features(sire PUBLIC cxx_std_17)
target_include_directories(sire
  PUBLIC
    ${aris_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
    ${ASSIMP_INCLUDE_DIRS}
  PRIVATE
		${CMAKE_SOURCE_DIR}/third_party/clarabel/include
)

# 链接库
target_link_libraries(sire
  PRIVATE
    sire_core
    sire_log
    sire_server
    sire_middleware
    sire_integrator
    sire_simulator
    sire_sensor
    sire_physics
    sire_actuator
    Threads::Threads
    ${aris_LIBRARIES}
    ${PUB_RELY_LINK_LIB} # 继承第三方依赖
    ${PRI_RELY_LINK_LIB}
)

# 设置模块输出名称和路径
set_target_properties(sire PROPERTIES
  OUTPUT_NAME "sire"
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${PYTHON_MODULE_OUTPUT_DIR}
)
if(WIN32)
  set_target_properties(sire PROPERTIES SUFFIX ".pyd")
else()
  set_target_properties(sire PROPERTIES SUFFIX ".so")
endif()

# # 复制运行时依赖的 DLL 到输出目录（Windows 特有）
# add_custom_command(TARGET sire_python POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy
# $<TARGET_FILE:sire_lib>
# ${PYTHON_MODULE_OUTPUT_DIR}/
# )

# 安装 Python 模块
install(TARGETS sire
  CONFIGURATIONS Release
  DESTINATION "${CMAKE_INSTALL_PREFIX}/python/release"
)

install(TARGETS sire
  CONFIGURATIONS Debug
  DESTINATION "${CMAKE_INSTALL_PREFIX}/python/debug"
)

# # 安装依赖 DLL（可选）
install(FILES
  $<TARGET_FILE:sire_lib>
  CONFIGURATIONS Debug
  DESTINATION "${CMAKE_INSTALL_PREFIX}/python/debug"
)

# # 安装依赖 DLL（可选）
install(FILES
  $<TARGET_FILE:sire_lib>
  CONFIGURATIONS Release
  DESTINATION "${CMAKE_INSTALL_PREFIX}/python/release"
)

INSTALL_DLL(DESTINATION ${CMAKE_INSTALL_PREFIX}/python/${CMAKE_BUILD_TYPE_LOWER})
