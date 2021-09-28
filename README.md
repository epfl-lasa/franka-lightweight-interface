# Franka Lightweight Interface

This package is a lightweight interface to connect to the Franka Panda robot, receive its state and send torques commands
to the internal controller. It is made to be system agnostic (not relying on a ROS installation) and uses a ZMQ based
communication network, encoding state and command data using `state_representation` and `clproto` from
[control libraries](https://github.com/epfl-lasa/control_libraries).

The design philosophy is to have two asynchronous processes which communicate over a common protocol:
- A realtime robot control interface (referred to as the _server_)
- A control loop (referred to as the _client_)

The server runs a simple internal controller that broadcasts the robot state and listens to commands to forward
to the robot. The client receives the robot state, calculates a desired control value in an asynchronous fashion,
and then sends the command to the server.


## Quick start

- [Preprocess](#preprocess)
- [Installation](#installation)
- [Connecting to the robot](#connecting-to-the-robot)
- [Robot IPs](#robot-ips)
- [Running the interface](#running-the-interface)
- [Examples](#examples)
- [Authors / Maintainers](#authors--maintainers)

## Preprocess

The Franka robot requires a realtime kernel for the server process to work properly.
To install one on your computer you can use a patched kernel following instructions [here](https://chenna.me/blog/2020/02/23/how-to-setup-preempt-rt-on-ubuntu-18-04/).
Any kernel is working, but we recommend using one closed to the version currently installed on your computer.
For example, on Ubuntu 18.04 a kernel v5.4.78 would work with the associated RT patch would work.
Note that not all the available kernels are patched so be sure to select that have an associated RT patch available.

## Installation

After the preprocess steps, you need to install libZMQ with C++ bindings, which in turn depends on libsodium and
libzmq3.

```bash
sudo apt-get update && sudo apt-get install -y \
  libsodium-dev \
  libzmq3-dev

# install cppzmq bindings
wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz
tar -xzf cppzmq-4.7.1.tar.gz
cd cppzmq-4.7.1
mkdir build
cd build
cmake .. -DCPPZMQ_BUILD_TESTS=OFF
sudo make -j4 install
cd ../..
rm -rf cppzmq*
```

You will also need to install `state_representation` and `clproto`
from [control libraries](https://github.com/epfl-lasa/control_libraries). The encoding library `clproto` also
requires [Google Protobuf](https://github.com/protocolbuffers/protobuf/tree/master/src) to be installed.

```bash
# install control library state representation
git clone -b develop --depth 1 https://github.com/epfl-lasa/control_libraries.git
cd control_libraries/source
sudo ./install.sh --no-controllers --no-dynamical-systems --no-robot-model --auto

# install clproto protobuf bindings
cd ../../control_libraries/protocol
RUN sudo ./install.sh && sudo ldconfig
```

To install this project, first clone the repository with the recursive option to
download [libfranka](https://frankaemika.github.io/docs/libfranka.html) and 
[network-interfaces](https://github.com/aica-technology/network-interfaces), added as a submodules:

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

### Install `libfranka`

The building process relies on cmake. You first need to build and
install [libfranka](https://frankaemika.github.io/docs/libfranka.html) following the recommendation from the website.
First download the required packages:

```bash
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libtool
```

Then build and install the library

```bash
cd lib/libfranka && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j && sudo make install -j && sudo ldconfig
```

### Build the interface

Finally, build the interface with:

```bash
cd franka_lightweight_interface
mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j && sudo ldconfig
```

Optionally, one can run `sudo make install` before `sudo ldconfig` to install the interface to `/usr/local/bin`.

## Connecting to the robot

The robot proposes two type of connections using ethernet cables, one is connected to the control box and one directly
to the robot. The latest is simply used to access the graphical interface to unlock the robot joints. For using the
robot with this library, you should connect to the control box only.

Connect the control box ethernet cable and follow the instructions to set up the connection with the proper IP address.
This is detailed
on [libfranka under Linux workstation network configuration](https://frankaemika.github.io/docs/getting_started.html).

You can then access the web interface that allows to unlock the joints of the robot on https://<robot-ip>.

## Robot IPs

There are currently two Franka Panda robots:

- Franka Papa, with IP `172.16.0.2` and ID `16`
- Franka Quebec 17, with IP `172.17.0.2` and ID `17`

## Running the interface

To start the interface, first unlock the robot joints using the web interface. The LEDs on the robot change to blue.
Then simply run the interface with your desired robot ID (either `16` or `17`) and optionally a prefix that allows to
set the robot name and joint names:

```bash
cd build && ./franka_lightweight_interface <robot-id> <robot-prefix>
```

In case the controller stops, due to violation of the velocities or efforts applied on the robot, you can push on the
emergency stop button, which turns the LEDs to white and unlock it to bring it back to blue. The controller is
automatically restarted to accept new commands.

## Examples

See [here](examples/README.md) for examples on how to use the interface to control the robot.

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Enrico Eberhard ([enrico.eberhard@epfl.ch](mailto:enrico.eberhard@epfl.ch))
- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))
- Baptiste Busch ([baptiste.busch@epfl.ch](mailto:baptiste.busch@epfl.ch))
