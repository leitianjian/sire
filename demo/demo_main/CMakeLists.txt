file(GLOB SOURCES *.h *.cpp)
set(sire_demo "demo_main")
set(CONFIG_EXISTS ON)
set(INSTALL_DESTINATION demo/${CMAKE_BUILD_TYPE_LOWER}/${sire_demo})

add_executable(${sire_demo} ${SOURCES})
target_compile_features(${sire_demo} PUBLIC cxx_std_17)
target_include_directories(${sire_demo}
	PRIVATE
		${aris_INCLUDE_DIRS}
		${PROJECT_BINARY_DIR}
		${hpp-fcl_INCLUDE_DIRS}
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
		${PUB_RELY_LINK_LIB}
		PRIVATE ftxui::screen
		PRIVATE ftxui::dom
		PRIVATE ftxui::component # Not needed for this example.
)

install(
	TARGETS
		sire_lib
		${sire_demo}
	DESTINATION
		${INSTALL_DESTINATION}
)

set(CONFIG_FILES "box_sliding.xml")

if(${CONFIG_EXISTS})
	install(FILES ${CONFIG_FILES} DESTINATION ${INSTALL_DESTINATION})
endif()

INSTALL_DLL(DESTINATION ${INSTALL_DESTINATION})

	# if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${sire_demo}/resource)
	# install(DIRECTORY resource DESTINATION demo/${sire_demo})
	# install(DIRECTORY resource DESTINATION resource/${sire_demo})
	# endif()