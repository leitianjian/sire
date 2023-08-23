file(GLOB SOURCES *.h *.cpp)
set(sire_demo "demo_server_controller_sensor")
set(CONFIG_EXISTS ON)
add_executable(${sire_demo} ${SOURCES})
target_compile_features(${sire_demo} PUBLIC cxx_std_17)
target_include_directories(${sire_demo} 
	PRIVATE 
		${aris_INCLUDE_DIRS} 
		${PROJECT_BINARY_DIR} 
		${hpp-fcl_INCLUDE_DIRS}
	)
target_link_libraries(${sire_demo}
	PRIVATE
		sire_core 
		sire_server 
		sire_middleware 
		sire_transfer 
		sire_integrator
		sire_sensor
		${PUB_RELY_LINK_LIB}
	)
# DEBUG INSTALL 
install(
	TARGETS 
		sire_lib 
		${sire_demo} 
	CONFIGURATIONS
		Debug
	DESTINATION 
		demo/debug/${sire_demo}
	)
install(
	FILES 
		"${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll" 
	CONFIGURATIONS
		Debug
	DESTINATION 
		demo/debug/${sire_demo}
)
if(${CONFIG_EXISTS})
	install(DIRECTORY config CONFIGURATIONS Debug DESTINATION demo/debug/${sire_demo})
endif()

# RELEASE INSTALL
install(
	TARGETS 
		sire_lib 
		${sire_demo} 
	CONFIGURATIONS
		Release
	DESTINATION 
		demo/release/${sire_demo}
	)
install(
	FILES 
		"${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll" 
	CONFIGURATIONS
		Release
	DESTINATION 
		demo/release/${sire_demo}
	)

# Copy config to install directory
if(${CONFIG_EXISTS})
	install(DIRECTORY config CONFIGURATIONS Release DESTINATION demo/release/${sire_demo})
endif()