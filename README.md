# SIRE 使用介绍
## 编译流程

### clone项目
#### 1. clone本项目 
``` 
git clone https://github.com/leitianjian/sire.git 
```
#### 2. 下载更新子模块
```
cd ArisSim
git submodule update --init --recursive
```
### 2. 使用vcpkg安装第三方库
#### 2.1 首先安装assimp库，最好是使用vcpkg安装，vcpkg详细内容见以下地址:
```
 https://github.com/microsoft/vcpkg 
```
使用vcpkg安装assimp的库：
```
.\vcpkg.exe install assimp:x64-windows
```
#### 2.2 手动安装hpp-fcl库
- 首先clone hpp-fcl库
```
git clone https://github.com/humanoid-path-planner/hpp-fcl 
```
- 编译hpp-fcl，在 ``third-parties/hpp-fcl`` 路径下，使用vs打开
在 项目-CMake设置中，设置CMAKE命令参数
```
-DHPP_FCL_HAS_QHULL=True
-DBUILD_TESTING=False
-DBUILD_PYTHON_INTERFACE=False
```
- 使用vcpkg安装boost以及qhull 库
```
  .\vcpkg.exe install boost:x64-windows
  .\vcpkg.exe install boost:x64-windows
```
- 生成 hpp-fcl 之后，默认安装位置在 `` ./out/install/x64-Debug``

### 3 编译sire，安装sire
#### 3.1 修改aris
本项目对原来的aris库进行了一点改变，应该将aris进行merge
首先，打开在aris文件夹下，打开 git bash，
使用命令 
``` 
git remote add ltj_aris https://github.com/leitianjian/aris.git
```
如果这个命令报错，使用 ``git init`` 命令初始化即可
之后使用
```
git remote -v
```
查看是否添加成功，出现版本号即为成功。
其次，使用
```
git fetch ltj_aris sensor
git merge ltj_aris/sensor
```
再重新安装aris即可，查看生成 aris 的文件夹名称，例如`` aris-2.3.4.221021 ``

#### 3.2 安装sire
通过CMAKE命令参数，设置各个库的位置，aris库通常在C盘文件下,hpp-fcl在安装路径下进行寻找
将新生成的 aris 库，添加在路径下
例如：
```
-DTARGET_ARIS_PATH=C:/aris/aris-2.3.4.221021 
-DTARGET_HPP_FCL_PATH = D:/third-parties/hpp-fcl/out/install/x64-Debug
```
安装sire，默认安装位置在 ``C:\sire `` 下

## 使用方法
- 代码文件需要的地方使用 `` #include <sire.hpp>``
- CMakeLists.txt 需要使用 ``find_package()``指令进行查找，查找方法与aris一致

示例：
``` CMAKE
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


