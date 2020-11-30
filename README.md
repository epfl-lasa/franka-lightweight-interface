# Franka Lightweight Interface

This package is a lightweight interface to connect to the robot, receive its state and send torques commands to the internal controller. It is made to not use ROS and relies on a ZMQ based communication process. The internal controller is a simple control loop that reads the state of the robot & the torque command and send the latest to the robot.

## Preprocess

Franka robot requires a realtime kernel to work properly. To install one on your computer you can use a patched kernel following instructions [here](https://chenna.me/blog/2020/02/23/how-to-setup-preempt-rt-on-ubuntu-18-04/). Any kernel is working, we recommend to use one closed to the version currently installed on your computer. For example, on Ubuntu 18.04 a kernel v5.4.78 would work with the associated RT patch would work. Note that all the available kernels are not patched so be sure to select that have an associated RT patch available.

## Installation

To install this library, first clone the repository with the recursive option to download [libfranka](https://frankaemika.github.io/docs/libfranka.html), added as a submodule:

```bash
git clone --recurse-submodules https://github.com/epfl-lasa/franka_lightweight_interface.git
```

of for ssh cloning:

```bash
git clone --recurse-submodules git@github.com:epfl-lasa/franka_lightweight_interface.git
```

In case you already cloned the repository you can use:

```bash
git submodule init && git submodule update
```

The building process relies on cmake. You first need to build and install [libfranka](https://frankaemika.github.io/docs/libfranka.html) following the recomendation from the website. First download the required packages:

```bash
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libtool
```

Then build and install the library

```bash
cd lib/libfranka && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j && sudo make install && sudo ldconfig
```


You also need to install libZMQ with C++ bindings, which in turn depends on libsodium. 

```bash
# install libsodium
RUN wget https://download.libsodium.org/libsodium/releases/libsodium-1.0.18.tar.gz \
  && tar -xzf libsodium-1.0.18.tar.gz \
  && cd libsodium-1.0.18 \
  && ./autogen.sh \
  && ./configure \
  && make check \
  && sudo make install \
  && sudo ldconfig \
  && cd .. \
  && rm -rf libsodium*

# install base libzmq
RUN wget https://github.com/zeromq/libzmq/releases/download/v4.3.3/zeromq-4.3.3.tar.gz \
  && tar -xzf zeromq-4.3.3.tar.gz \
  && cd zeromq-4.3.3 \
  && mkdir build \
  && cd build \
  && cmake .. -DWITH_PERF_TOOL=OFF -DZMQ_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
  && sudo make -j4 install \
  && cd ../.. \
  && rm -rf zeromq*

# install cppzmq bindings
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz \
  && tar -xzf cppzmq-4.7.1.tar.gz \
  && cd cppzmq-4.7.1 \
  && mkdir build \
  && cd build \
  && cmake .. -DCPPZMQ_BUILD_TESTS=OFF \
  && sudo make -j4 install \
  && cd ../.. \
  && rm -rf cppzmq*
```


Finally build the interface with:

```bash
cd franka_lightweight_interface
mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j
```

## Connecting to the robot

The robot proposes two type of connections using ethernet cables, one is connected to the control box and one directly to the robot. The latest is simply used to access the graphical interface to unlock the robot joints. For using the robot with this library, you should connect to the control box only.

Connect the control box ethernet cable and follow the instructions to setup the connection with the proper IP adress. This is detailed on [libfranka under Linux workstation network configuration](https://frankaemika.github.io/docs/getting_started.html). The IP should be fixed and by default correspond to ` 172.16.0.1`.

You can then access the web interface that allows to unlock the joints of the robot on https://172.16.0.2 (this is the default adress. Replace it by the correct IP if you have changed it on the robot).

## Running the interface

To start the interface, first unlock the robot joints using the web interface. The leds on the robot change to blue. Then simply run:

```bash
cd build && ./franka_lightweight_interface
```

The interface is build to connect to the robot at `172.16.0.2`, to change it, modify it in the [main.cpp](src/main.cpp) file and rebuild. In case the controller stops, due to violation of the velocities or efforts applied on the robot. You can push on the emergency button, which turns the leds to white and unlock it to bring it back to blue. The controller is automatically restarted to accept new commands.