# Aris_sim 使用介绍

## 编译流程

注：我们只测试了windows msvc vs 2017/2019

1. Clone项目并用IDE打开
2. 本项目依赖[ARIS](https://github.com/py0330/aris)、[FCL](https://github.com/flexible-collision-library/fcl)、[CCD](https://github.com/danfis/libccd)、[ASSIMP](https://github.com/assimp/assimp) 可以通过TARGET_XXX_PATH指定查找位置（指定后会加入CMAKE_PREFIX_PATH变量中）
3. INSTALL位置和ARIS相同为C:\aris_sim目录下
4. 碰到问题，请发送邮件至leitianjian@outlook.com

## 使用方法

注：和ARIS一致

1. 代码文件需要的地方使用 `#include <aris_sim.hpp>`

2. CMakeLists.txt需要使用`find_package()`指令进行查找，查找方法与aris一致

   示例：

   ```cmake
   set(ARIS_SIM_INSTALL_PATH C:/aris_sim CACHE PATH "Aris install path") # 设置默认查找位置 C:\aris_sim
   
   if(EXISTS ${ARIS_SIM_INSTALL_PATH})
   	message(STATUS "Directory to search aris_sim at ${ARIS_SIM_INSTALL_PATH}")
   	list(APPEND CMAKE_PREFIX_PATH ${ARIS_SIM_INSTALL_PATH})
   else()
   	message(WARNING "File/Directory at variable ARIS_SIM_INSTALL_PATH not exists!")
   endif()
   find_package(aris_sim REQUIRED)
   
   include_directories(${aris_sim_INCLUDE_DIRS})
   
   target_link_libraries(${PROJECT_NAME} ${aris_sim_LIBRARIES})
   ```

   