# Sire

简短的项目描述，介绍项目的目的和功能。

## 目录

- [背景](#背景)
- [功能](#功能)
- [安装](#安装)
  - [Windows](#windows)
  - [Linux](#linux)

## 背景

暂无

## 功能

暂无

## 安装

### Windows

#### 1. 环境要求

- Windows 10 或更高版本
- CMake 版本 3.18.0 或更高
- vcpkg 包管理工具
- python3.10

#### 2. 安装步骤

1. **安装 CMake**

   下载并安装 [CMake](https://cmake.org/download/)，确保在系统路径中添加 CMake。

2. **安装 vscode （推荐）**

   我们推荐您在windows系统上使用vscode对项目进行编译开发

3. **安装 vcpkg**

   克隆 vcpkg 仓库并安装：

   ```bash
   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   .\bootstrap-vcpkg.bat
   ```

4. **克隆项目仓库**

   从github克隆项目到本地

   ```bash
   git clone https://github.com/leitianjian/sire.git
   git checkout dev
   git submodule update --init --recursive
   ```

5. **执行安装脚本**

   在```/sire/scripts/tools/build_scripts```路径下，可以直接执行```python build_all.py```脚本，即可一键安装

   需注意windows环境下需要您指定```vcpkg.cmake```的绝对路径，请在```python build_all.py```和```build_third_parties.py```中分别修改为您的安装路径

   如需单独重新安装某个依赖库，可以参考```build_third_parties.py```中的指令，单独进行安装（或者直接将脚本命令从命令数组中删除）

### Linux

#### 1.环境要求

- Ubuntu20 或更高版本
- CMake 版本 3.18.0 或更高
- vcpkg 包管理工具
- python3.10

#### 2. 安装步骤

1. **安装 CMake**

   下载并安装 [CMake](https://cmake.org/download/)，确保在系统路径中添加 CMake。

2. **克隆项目仓库**

   从github克隆项目到本地

   ```bash
   git clone https://github.com/leitianjian/sire.git
   git checkout dev
   git submodule update --init --recursive
   ```

3. **安装依赖三方库**

   在安装sire之前，你需要将其依赖的aris，hpp-fcl，stduuid以及msgpack-c安装到你的环境中

   ~~~shell
   #安装一些依赖环境
   apt-get install -y \
         libeigen3-dev \
         libboost-thread-dev \
         libboost-date-time-dev \
         libboost-filesystem-dev \
         libassimp-dev \
         liboctomap-dev \
         libqhull-dev
         
   #安装airs
   cd aris
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release \
   make -j8 install
   
   #安装hpp-fcl
   cd hpp-fcl
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=False -DBUILD_TESTING=False \
   make -j8 install
   ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
   
   #安装stduuid
   cd stduuid
   mkdir build && cd build
   cmake ..
   make -j8 install
   
   #安装msgpack-c(if needed)
   git clone https://github.com/msgpack/msgpack-c.git && git checkout cpp_master
   cd msgpack-c && mkdir build && cd build
   cmake ..
   make -j8 install
   ~~~

   上述步骤均可在提供的脚本内自动执行，也可手动操作

4. **安装sire**

   安装完依赖库之后，就可以安装sire

   ~~~shell
   cd sire && mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_DEMO=True -DBUILD_TEST=False -DTARGET_ARIS_PATH=/usr/aris
   make -j8 install
   ~~~

   完成上述步骤后即可成功安装sire
   也可以直接使用项目根目录提供的Dockerfile直接完成环境搭建工作