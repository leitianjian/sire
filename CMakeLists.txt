cmake_minimum_required(VERSION 3.12)
project(ArisSim)

# use c++ 17 standard
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH bin)
set(USE_EIGEN3 OFF)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message("Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

if (MSVC)
  add_compile_options(/bigobj)  
else ()
  add_compile_options(-Wa,-mbig-obj)
endif ()

if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/aris_sim/aris_sim-1.0")
	add_custom_target(uninstall rm -r /usr/aris)
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/aris_sim/aris_sim-1.0")
endif(WIN32)
add_definitions(-DARIS_SIM_INSTALL_PATH="${CMAKE_INSTALL_PREFIX}")

## set rely libs ##
# std & socker rely #
if(UNIX)
	set(RELY_LINK_LIB atomic pthread stdc++fs dl)#pthread is needed for concurrency
	set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config)
elseif(WIN32)
	set(RELY_LINK_LIB "ws2_32")#ws2_32 is needed for socket
endif()

# set aris and kaanh path
if(UNIX) 
    add_definitions(-DUNIX)
    set(ARIS_INSTALL_PATH /usr/aris CACHE PATH "Aris install path")
    set(KAANH_INSTALL_PATH /usr/kaanh CACHE PATH "Kaanh install path")
endif(UNIX)
if(WIN32)
    set(ARIS_INSTALL_PATH C:/aris CACHE PATH "Aris install path")
endif(WIN32)

# linux find kaanh and aris
if(UNIX)
	# find aris
	if(EXISTS ${ARIS_INSTALL_PATH})
		message(STATUS "Directory to search aris at ${ARIS_INSTALL_PATH}")
		list(APPEND CMAKE_PREFIX_PATH ${ARIS_INSTALL_PATH})
	else()
		message(WARNING "File/Directory at variable ARIS_INSTALL_PATH not exists!")
	endif()
	find_package(aris REQUIRED)
	if(aris_FOUND)
		message(STATUS "Aris found at ${aris_DIR}")
		list(APPEND RELY_INCL_DIR ${aris_INCLUDE_DIRS})
		list(APPEND RELY_LINK_LIB ${aris_LIBRARIES})
	endif()

	# find kaanh
	if(EXISTS ${KAANH_INSTALL_PATH})
		message(STATUS "Directory to search kaanh at ${KAANH_INSTALL_PATH}")
		list(APPEND CMAKE_PREFIX_PATH ${KAANH_INSTALL_PATH})
	else()
		message(WARNING "File/Directory at variable KAANH_INSTALL_PATH not exists!")
	endif()
	find_package(kaanh REQUIRED)
	if(kaanh_FOUND)
		message(STATUS "Kaanh found at ${kaanh_DIR}")
		list(APPEND RELY_INCL_DIR ${kaanh_INCLUDE_DIRS})
		list(APPEND RELY_LINK_LIB ${kaanh_LIBRARIES})
	endif()
endif(UNIX)

# windows only find aris
if(WIN32)
	# find aris
	if(EXISTS ${ARIS_INSTALL_PATH})
		message(STATUS "Directory to search aris at ${ARIS_INSTALL_PATH}")
		list(APPEND CMAKE_PREFIX_PATH ${ARIS_INSTALL_PATH})
	else()
		message(WARNING "File/Directory at variable ARIS_INSTALL_PATH not exists!")
	endif()
	find_package(aris REQUIRED)
	if(aris_FOUND)
		message(STATUS "Aris found at ${aris_DIR}")
		list(APPEND RELY_INCL_DIR ${aris_INCLUDE_DIRS})
		list(APPEND RELY_LINK_LIB ${aris_LIBRARIES})
	endif()
endif(WIN32)


# set ccd and fcl path
set(CCD_INSTALL_PATH "" CACHE PATH "Ccd install path")
set(FCL_INSTALL_PATH "" CACHE PATH "Fcl install path")
set(ASSIMP_INSTALL_PATH "" CACHE PATH "Assimp install path")

# find ccd
if(EXISTS ${CCD_INSTALL_PATH})
	message(STATUS "Directory to search ccd at ${CCD_INSTALL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${CCD_INSTALL_PATH})
else()
	message(WARNING "File/Directory at variable CCD_INSTALL_PATH not exists!")
endif()
find_package(ccd CONFIG REQUIRED)
if(ccd_FOUND)
	message(STATUS "Libccd found at ${ccd_DIR}")
	list(APPEND RELY_INCL_DIR ${CCD_INCLUDE_DIRS})
	list(APPEND RELY_LINK_LIB ${CCD_LIBRARIES})
endif()

# find fcl
if(EXISTS ${FCL_INSTALL_PATH})
	message(STATUS "Directory to search fcl at ${FCL_INSTALL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${FCL_INSTALL_PATH})
else()
	message(WARNING "File/Directory at variable FCL_INSTALL_PATH not exists!")
endif()
find_package(fcl CONFIG REQUIRED)
if(fcl_FOUND)
	message(STATUS "Libfcl found at ${fcl_DIR} ${FCL_LIBRARIES}")
	list(APPEND RELY_INCL_DIR ${FCL_INCLUDE_DIRS})
	list(APPEND RELY_LINK_LIB ${FCL_LIBRARIES})
endif()

# find Assimp
if(EXISTS ${ASSIMP_INSTALL_PATH})
	message(STATUS "Directory to search Assimp at ${ASSIMP_INSTALL_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${ASSIMP_INSTALL_PATH})
else()
	message(WARNING "File/Directory at variable ASSIMP_INSTALL_PATH not exists!")
endif()
find_package(assimp CONFIG REQUIRED)
if(assimp_FOUND)
	message(STATUS "Assimp found at ${assimp_DIR} ${ASSIMP_INCLUDE_DIRS}")
	list(APPEND RELY_INCL_DIR ${ASSIMP_INCLUDE_DIRS})
	list(APPEND RELY_LINK_LIB ${ASSIMP_LIBRARIES})
endif()

# find Eigen3
if(DEFINED USE_EIGEN3)
	find_package(Eigen3 REQUIRED NO_MODULE)
	list(APPEND RELY_INCL_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

foreach(target collision simulator)
	file(GLOB HEADERS include/${target}/*.hpp include/${target}/*.h)
	file(GLOB SOURCES src/${target}/*.cpp src/${target}/*.c src/${target}/*.hpp src/${target}/*.h ${HEADERS})
	add_library(aris_sim_${target} OBJECT ${HEADERS} ${SOURCES})
	target_compile_definitions(aris_sim_${target} PRIVATE aris_sim_lib_EXPORTS)
	target_include_directories(aris_sim_${target}
		PUBLIC 
			$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
		PRIVATE
			${PROJECT_SOURCE_DIR}/src
			${PROJECT_BINARY_DIR}
			${RELY_INCL_DIR}
	)
	
	set_target_properties(aris_sim_${target} PROPERTIES PUBLIC_HEADER "${HEADERS}")
	list(APPEND aris_sim_LIBRARIES optimized aris_sim::aris_sim_${target} debug aris_sim::debug::aris_sim_{target})
	list(APPEND ARIS_SIM_LINK_LIB aris_sim_${target})
	list(APPEND ARIS_SIM_OBJECTS $<TARGET_OBJECTS:aris_sim_${target}>)
	list(APPEND ARIS_SIM_HEADERS include/${target})
	
	install(FILES ${HEADERS} DESTINATION include/${target})
endforeach(target)

add_library(aris_sim_lib SHARED ${ARIS_SIM_OBJECTS})
# generate_export_header(aris_sim_lib EXPORT_MACRO_NAME ARIS_SIM_API)
target_link_libraries(aris_sim_lib PUBLIC ${RELY_LINK_LIB})
target_include_directories(aris_sim_lib
	PUBLIC 
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${RELY_INCL_DIR}
)

set(aris_sim_LIBRARIES optimized aris_sim::aris_sim_lib debug aris_sim::debug::aris_sim_lib)

install(TARGETS aris_sim_lib EXPORT aris_sim-targets-debug CONFIGURATIONS Debug DESTINATION "lib/debug")
install(TARGETS aris_sim_lib EXPORT aris_sim-targets-release CONFIGURATIONS Release DESTINATION "lib/release")
set(ARIS_SIM_LINK_LIB aris_sim_lib)

## install exports ##
install(EXPORT aris_sim-targets-debug NAMESPACE aris_sim::debug:: FILE aris_sim-targets-debug.cmake DESTINATION "share/aris_sim/cmake")
install(EXPORT aris_sim-targets-release NAMESPACE aris_sim:: FILE aris_sim-targets-release.cmake DESTINATION "share/aris_sim/cmake")
install(FILES include/aris_sim.hpp DESTINATION include)
