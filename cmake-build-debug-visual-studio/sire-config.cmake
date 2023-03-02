
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was sire-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

#################################################################################### 

set(RELY_LINK_DIR "")
link_directories(${RELY_LINK_DIR})

set_and_check(sire_INCLUDE_DIRS "C:/sire/sire-1.0/include")
set(sire_LIBRARIES "optimized;sire::sire_lib;debug;sire::debug::sire_lib;H:/contact/6.refer_project/hpp-fcl/out/install/x64-Debug/lib/hpp-fcl.lib;optimized;aris::aris_lib;debug;aris::debug::aris_lib;ws2_32")
if(UNIX)
	set(sire_LIBRARIES -Wl,--start-group ${sire_LIBRARIES} -Wl,--end-group)
endif(UNIX)


include("${CMAKE_CURRENT_LIST_DIR}/sire-targets-release.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/sire-targets-debug.cmake")
