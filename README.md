# SOEM Interface

## Overview
This software package serves as a C++ interface for one or more EtherCAT devices running on the same bus.
The lower level EtherCAT communication is handled by the [SOEM](https://github.com/OpenEtherCATsociety/soem) library.

The soem_interface has been developed on Ubuntu 18.04 LTS with ROS Melodic.

The source code is released under the GPLv3 license.
A copy of the license is available in the *COPYING* file.

**Author:** Markus Staeuble

**Affiliation:** Robotic Systems Lab - ETH Zurich

**Maintainer:** Johannes Pankert, johannes.pankert@mavt.ethz.ch

**Contributors:** Johannes Pankert, Jonas Junger, Lennart Nachtigall


## Installation

### Dependencies
#### Catkin Packages

| Repo           | url                                                  | license      | content            |
|:--------------:|:----------------------------------------------------:|:------------:|:------------------:|
| message_logger | https://github.com/leggedrobotics/message_logger.git | BSD 3-Clause | simple log streams |

#### System Dependencies (Ubuntu 18.04 LTS)
- [ROS Melodic](https://wiki.ros.org/melodic) (full installation)
- [catkin](https://wiki.ros.org/catkin)

### Building from Source

To build the library from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/leggedrobotics/message_logger.git
	git clone https://github.com/leggedrobotics/soem_interface.git
	cd ../
	catkin build soem_interface

To build the examples, execute the following command inside of your catkin workspace:
	
	catkin build soem_interface_examples
	
## Classes

#### EthercatSlaveBase
This is an abstract base class for an ethercat slave. The ethercat slave class holds a non owning reference to the EthercatBusBase it is on (bus_). 
EthercatSlaves should stage their tx messages before writing to the ethercat bus in the updateWrite() method using the bus_ writeRxPdo() method.
Similarly, the slaves can retrieve the buffered read messages from the bus with the readTxPdo() method in updateRead().

#### EthercatBusBase
This class represents a physical ethercat bus containing multiple ethercat slaves. 
It manages the slaves on the bus using the methods from soem.  With updateRead() and updatewrite() the staged slave messages are written/read to/from all the slaves on the bus simultaneously. 
EthercatSlaves can be added with addSlave(). After all the slaves have been added the startup() method to actually start the communication of the bus.

#### EthercatBusManagerBase
If multiple buses are connected to the same master then the buses are managed by the EthercatBusManagerBase.

## Note
Due to the current pandemic we could not test this version of the soem_interface.

Tests will be conducted as soon as possible.

