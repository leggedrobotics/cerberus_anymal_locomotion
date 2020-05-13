# CERBERUS ANYmal Locomotion
This repository contains the software that can be used to control the ANYmal model in the  [DARPA Subterranean Challenge Virtual Competition Simulator](https://github.com/osrf/subt/wiki).

**Authors & Maintainers**:
  * Marco Tranzatto
  * Samuel Zimmermann
  * Timon Homberger

**Affiliation:** [Robotic Systems Lab - ETH Zurich](https://rsl.ethz.ch/the-lab.html)

[![CERBERUS ANYmal](doc/cerberus_anymal.gif)](doc/cerberus_anymal.gif)

## License
This software is released under a [BSD 3-Clause license](LICENSE).

## Publications
If you use this work in an academic context, please cite the following publications:

`Coming soon ...`

## Dependencies
The following dependencies are needed to run the ANYmal locomotion controller.
* catkin workspace setup for the SubT (Ignition) Simulation. See [SubT (Ignition) Simulation](https://github.com/osrf/subt/wiki/Catkin%20System%20Setup).
* catkin_tools package. See [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

## Instructions
Create the `anymal_locomotion_ws` workspace and clone the cerberus_anymal_locomotion repo:
```
mkdir -p ~/anymal_locomotion_ws/src && cd ~/anymal_locomotion_ws/src

git clone git@github.com:leggedrobotics/cerberus_anymal_locomotion.git # or over https: git clone https://github.com/leggedrobotics/cerberus_anymal_locomotion.git

cd cerberus_anymal_locomotion/

git submodule update --init --recursive
```

Configure and build the workspace. We need to overlay the `subt_ws` workspace because of protobuf version mismatch between one of our dependency (tensorflow-cpp) and the version used by Ignition Gazebo.
```
source /opt/ros/melodic/setup.bash

cd ~/anymal_locomotion_ws/

catkin config --extend ~/subt_ws/install/

catkin build cerberus_anymal_b_control_1
```

Launch the ANYmal locomotion controller:
```
source ~/anymal_locomotion_ws/devel/setup.bash

roslaunch cerberus_anymal_b_control_1 cerberus_anymal_controller.launch
```

## Bugs & Issues
Please report bugs or issues using the [Issue Tracker](https://github.com/leggedrobotics/cerberus_anymal_locomotion/issues).
