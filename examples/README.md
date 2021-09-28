# Examples

This directory contains executables to showcase the intended use of the `franka_lightweight_interface` with simple
control loop clients.

## Installation

To run control loops in packaged applications, it is not required to install the whole `franka_lightweight_interface`
project as described in the top level README. The full installation including a real time kernel and `libfranka` is
only necessary for the robot server process. The client process running the control loop only needs to know the
communication protocol to be able to receive and send information via ZMQ sockets. This can be
seen in the simple [`joint_position_controller`](joint_position_controller.cpp), where the only `include`s are

```cpp
#include "network_interfaces/control_type.h"
#include "network_interfaces/zmq/network.h"
```

To install the `network_interface` headers for your own application, follow the installations steps
[here](https://github.com/aica-technology/network-interfaces).

## `joint_position_controller`

The [joint position controller](./joint_position_controller.cpp) example is given as a starting template for 
desing control loop clients. After importing the frankalwi network header, the common process

- Set up the ZMQ connection using `network_interfaces::zmq::configure_sockets()`
- Prepare the state and command messages of type `network_interfaces::zmq::StateMessage` and `network_interfaces::zmq::CommandMessage`
- Check for a new state message from the robot using `network_interfaces::zmq::receive(state, subscriber)`
- Calculate the desired command output given the state and some control logic
- Send the command packet back to the robot using `network_interfaces::zmq::send(command, publisher)`

The example controller is kept intentionally simple; the intention is for users to develop and contain control loop
clients in external projects or repositories following the given examples, and not to implement controllers for
production or experiments in this folder.
