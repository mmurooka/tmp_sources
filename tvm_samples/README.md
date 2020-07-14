# tvm_samples
Samples of [TVM](https://github.com/jrl-umi3218/tvm)

## Install

ROS is necessary.
```
$ mkdir -p ~/ros/ws_tvm/src
$ cd ~/ros/ws_tvm
$ wstool init src
$ wstool set -t src tmp_sources https://github.com/mmurooka/tmp_sources.git --git -y
$ wstool set -t src mc_rtc_data https://github.com/jrl-umi3218/mc_rtc_data --git -y
$ wstool update -t src

$ source /opt/ros/kinetic/setup.bash # or setup.bash in the other catkin workspace to be cascaded
$ catkin build -DCMAKE_BUILD_TYPE=Release

```

Add the following line to ~/.bashrc.
```
source ~/ros/ws_tvm/devel/setup.bash
```

### use C++17 for TVM in Ubuntu16.04
https://qiita.com/forno/items/11c4a0f8169d987f232b

1. install g++
```
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
$ sudo apt update
$ sudo apt install -y g++-9
$ echo "export CXX='g++-9'" >> ~/.bashrc
$ echo "export CC='gcc-9'" >> ~/.bashrc
$ g++-9 --version # check
```

2. install cmake
```
$ git clone https://gitlab.kitware.com/cmake/cmake.git
$ cd cmake
$ ./bootstrap
$ make -j5
$ sudo make install
$ cmake --version # check
```

## Samples

### Interactive IK
```
$ roslaunch tvm_samples tvm_ik_sample_jvrc1.launch
```

![](doc/images/tvm_ik_sample_jvrc1.png)
