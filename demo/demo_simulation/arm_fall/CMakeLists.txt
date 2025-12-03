file(GLOB SOURCES *.h *.hpp *.cpp)
set(sire_demo "demo_simulation_arm_fall")
set(CONFIG_EXISTS ON)
set(INSTALL_DESTINATION demo/${CMAKE_BUILD_TYPE_LOWER}/${sire_demo})

add_executable(${sire_demo} ${SOURCES})
target_compile_features(${sire_demo} PUBLIC cxx_std_17)
target_include_directories(${sire_demo}
	PRIVATE
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR}
		${coal_INCLUDE_DIRS}
		${ASSIMP_INCLUDE_DIRS}
)
target_link_libraries(${sire_demo}
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
		${PUB_RELY_LINK_LIB}
)

install(
	TARGETS
		sire_lib
		${sire_demo}
	DESTINATION
		${INSTALL_DESTINATION}
)

if(${CONFIG_EXISTS})
	install(DIRECTORY config DESTINATION ${INSTALL_DESTINATION})
endif()

INSTALL_DLL(DESTINATION ${INSTALL_DESTINATION})

# if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${sire_demo}/resource)
# install(DIRECTORY resource DESTINATION demo/${sire_demo})
# install(DIRECTORY resource DESTINATION resource/${sire_demo})
# endif()