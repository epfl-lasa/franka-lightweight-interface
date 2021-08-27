# Examples

This directory contains executables to showcase the intended use of the `franka_lightweight_interface` with simple
control loops.

## Installation

To run control loops in packaged applications, it is not required to install the whole `franka_lightweight_interface`
project as described in the top level README. Only the machine connected directly to the robot needs to be running the
interface with the full installation including a real time kernel and `libfranka`. The control loops on the other side
only needs to know the communication protocol to be able to receive and send information via ZMQ sockets. This can be
seen in the simple [`joint_position_controller`](joint_position_controller.cpp), where the only `include` is

```cpp
#include "frankalwi_proto/frankalwi_network.h"
```

To install the `frankalwi_proto` headers for your own application, follow the installations steps in the top level
README to install libZMQ with C++ bindings, `state_representation` and `clproto`. Afterwards, do the following:

```bash
git clone https://github.com/epfl-lasa/franka_lightweight_interface.git
cd franka_lightweight_interface
mkdir build && cd build && cmake -DFRANKALWI_PROTO_ONLY=On .. && sudo make install -j && sudo ldconfig
```

This installs the `frankalwi_proto` headers in your `/usr/local/include` and makes them available for your own project.

## `joint_position_controller`

TODO (explain controller, maybe mention that this is technically the wrong place for an executable and that they should
be in your own package)