# Examples

This directory contains executables to showcase the intended use of the `franka_lightweight_interface` with simple
control loop clients.

## Installation

To run control loops in packaged applications, it is not required to install the whole `franka_lightweight_interface`
project as described in the top level README. The full installation including a real time kernel and `libfranka` is
only necessary for the robot server process. The client process running the control loop only needs to know the
communication protocol to be able to receive and send information via ZMQ sockets. This can be
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

The [joint position controller](./joint_position_controller.cpp) example is given as a starting template for 
desing control loop clients. After importing the frankalwi network header, the common process

- Set up the ZMQ connection using `frankalwi::network::configure_sockets()`
- Prepare the state and command messages of type `frankalwi::proto::StateMessage` and `frankalwi::proto::CommandMessage`
- Check for a new state message from the robot using `frankalwi::network::poll(state, subscriber)`
- Calculate the desired command output given the state and some control logic
- Send the command packet back to the robot using `frankalwi::network::send(command, publisher)`

The example controller is kept intentionally simple; the intention is for users to develop and contain control loop
clients in external projects or repositories following the given examples, and not to implement controllers for
production or experiments in this folder.
