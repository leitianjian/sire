# SIRE 使用介绍

## 编译流程

注：我们只测试了windows msvc vs 2017/2019

### Clone 项目

1. Clone 本项目

   ```bash
   git clone git@github.com:leitianjian/ArisSim.git
   ```

2. 下载在本项目中的第三方依赖 hpp-fcl submodule

   ```bash
   cd ArisSim
   git submodule update --init --recursive
   ```

### 编译项目

本项目依赖[ARIS](https://github.com/py0330/aris)、HPP-FCL（在项目仓库中）、[ASSIMP](https://github.com/assimp/assimp) 可以通过TARGET_XXX_PATH指定查找位置（指定后会加入CMAKE_PREFIX_PATH变量中）

1. 首先需要编译hpp-fcl，在`third-parties/hpp-fcl`路径下，使用vs打开

   设置CMAKE命令参数

   ```bash
   -DHPP_FCL_HAS_QHULL=True
   -DBUILD_TESTING=False
   -DBUILD_PYTHON_INTERFACE=False
   ```

2. 安装 hpp-fcl 之后，默认安装位置在 `./out/install/x64-Debug`

3. 之后使用vs打开ArisSim

   通过CMAKE命令参数，设置各个库的位置

   ```bash
   -DTARGET_ARIS_PATH=C:/aris/aris-2.3.2.220803 
   -DTARGET_HPP_FCL_PATH=D:/leitianjian/Documents/code/ArisSim/third-parties/hpp-fcl/out/install/x64-Debug
   ```

4. 安装ArisSim 默认安装位置在 `C:\sire`下

5. 碰到问题，请发送邮件至leitianjian@outlook.com

## 使用方法

注：和ARIS一致

1. 代码文件需要的地方使用 `#include <sire.hpp>`

2. CMakeLists.txt需要使用`find_package()`指令进行查找，查找方法与aris一致

   示例：

   ```cmake
   # find Assimp
   set(TARGET_ASSIMP_PATH "" CACHE PATH "Assimp install path")
   if(EXISTS ${TARGET_ASSIMP_PATH})
   	message(STATUS "Directory to search Assimp at ${TARGET_ASSIMP_PATH}")
   	list(APPEND CMAKE_PREFIX_PATH ${TARGET_ASSIMP_PATH})
   else()
   	message(WARNING "File/Directory at variable TARGET_ASSIMP_PATH not exists!")
   endif()
   find_package(assimp REQUIRED)
   
   # find Hpp-fcl
   set(TARGET_HPP_FCL_PATH "" CACHE PATH "Hpp-fcl install path")
   if(EXISTS ${TARGET_HPP_FCL_PATH})
   	message(STATUS "Directory to search Assimp at ${TARGET_HPP_FCL_PATH}")
   	list(APPEND CMAKE_PREFIX_PATH ${TARGET_HPP_FCL_PATH})
   else()
   	message(WARNING "File/Directory at variable TARGET_HPP_FCL_PATH not exists!")
   endif()
   find_package(hpp-fcl REQUIRED)
   
   set(SIRE_INSTALL_PATH C:/sire CACHE PATH "Sire install path") # 设置默认查找位置 C:\sire
   if(EXISTS ${SIRE_INSTALL_PATH})
   	message(STATUS "Directory to search sire at ${SIRE_INSTALL_PATH}")
   	list(APPEND CMAKE_PREFIX_PATH ${SIRE_INSTALL_PATH})
   else()
   	message(WARNING "File/Directory at variable SIRE_INSTALL_PATH not exists!")
   endif()
   find_package(sire REQUIRED)
   
   include_directories(${SIRE_INSTALL_PATH})
   include_directories(${hpp-fcl_INCLUDE_DIRS})
   include_directories(${assimp_INCLUDE_DIRS})
   
   target_link_libraries(${PROJECT_NAME} ${sire_LIBRARIES})
   target_link_libraries(${PROJECT_NAME} assimp::assimp)
   target_link_libraries(${PROJECT_NAME} ${hpp-fcl_LIBRARIES})
   ```
   
   