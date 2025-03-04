add_library(sire_log OBJECT)
target_compile_features(sire_log PRIVATE cxx_std_11)
target_compile_definitions(sire_log 
  PRIVATE 
    -DAUTO_INITIALIZE_EASYLOGGINGPP 
  )
target_compile_options(sire_log
	PRIVATE 
		$<$<CXX_COMPILER_ID:MSVC>:/Zc:preprocessor>
		# $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wa,-mbig-obj>
	)
target_sources(sire_log 
	PRIVATE 
    "easylogging++.cc"
    "easylogging++.h"
    "easyloggingConfig.hpp"
    "easyloggingConfig.cpp"
	)
target_include_directories(sire_log
	PUBLIC
		$<INSTALL_INTERFACE:include> 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_BINARY_DIR} # sire_lib_EXPORTS
	)