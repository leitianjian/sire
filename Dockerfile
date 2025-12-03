FROM ubuntu:focal

# update source list
RUN sed -i s@/archive.ubuntu.com/@/mirrors.tuna.tsinghua.edu.cn/@g /etc/apt/sources.list \
      && apt-get update 
# set locale
RUN apt-get -y install locales \
      && sed -i 's/# \(en_US\.UTF-8 .*\)/\1/' /etc/locale.gen \
      && locale-gen
ENV LANG en_US.UTF-8 \ 
      LANGUAGE en_US:en \
      LC_ALL en_US.UTF-8 \
      TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
      && apt-get install -y \
      software-properties-common \
      build-essential \
      wget \
      make \
      git \
      curl \
      cmake \
      gcc \ 
      g++
# install hpp-fcl prerequisite.
RUN apt-get install -y \
      libeigen3-dev \
      libboost-thread-dev \
      libboost-date-time-dev \
      libboost-filesystem-dev \
      libassimp-dev \
      liboctomap-dev \
      libqhull-dev

# clone hpp-fcl
RUN cd ~ && mkdir code && cd code \
      && git clone https://github.com/humanoid-path-planner/hpp-fcl.git \
      && cd hpp-fcl \
      && git submodule update --init --recursive \
      && mkdir build \ 
      && cd build \ 
      && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=False -DBUILD_TESTING=False \
      && make -j8 install \
      && ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# clone aris from py branch
RUN cd ~/code \
      && git clone --single-branch --branch py https://github.com/py0330/aris.git \
      && cd aris \
      && mkdir build \
      && cd build \
      && cmake .. -DCMAKE_BUILD_TYPE=Release \
      && make -j8 install

# clone stduuid from py branch
RUN cd ~/code \
      && git clone https://github.com/mariusbancila/stduuid.git \
      && cd stduuid \
      && mkdir build \
      && cd build \
      && cmake ..\
      && make -j8 install

# clone msgpack-c from py branch
RUN cd ~/code \
      && git clone https://github.com/msgpack/msgpack-c.git \
      && git checkout cpp_master \
      && cd msgpack-c \
      && mkdir build \
      && cd build \
      && cmake ..\
      && make -j8 install

# clone sire and build install
RUN cd ~/code \
      && git clone --single-branch --branch dev https://github.com/leitianjian/sire.git \
      && cd sire \
      && mkdir build \
      && cd build \
      && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_DEMO=True -DBUILD_TEST=False -DTARGET_ARIS_PATH=/usr/aris \
      && make -j8 install

CMD bash