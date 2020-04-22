# SmallDrop
SmallDrop is a project being developed during my master thesis in medical robotics. It consists on a bioprinting system for *in situ* printing directly on burn wounds.

The main system is composed of three sub-systems:

- A 7 DOF robotic arm, named Panda from Franka Emika.
- A D415 Intel RealSense Depth camera.
- A custom print head based on a DIY syringe pump model.

The whole project is ROS-based and is being developed using ROS Melodic on a Linux Mint 19.2 MATE 64-bit. It has a simulation component based on the Gazebo simulator, and it also runs on the real robot. The robot is controlled via custom controllers.

## Package list

This project is composed of several ROS packages that communicate with each other. Some external package dependencies were added to the project as git submodules.

**Main packages**

- **smalldrop:** Project metapackage.
- **smalldrop_bioprint:** Responsible for controlling the whole system.
- **smalldrop_robot_arm:** Responsible for the robot arm control for the real robot and Gazebo simulation.
- **smalldrop_msgs:** Responsible for declaring all messages, services and actions for the project.
- **smalldrop_rviz:** Responsible for handling rviz configurations and visualisation displays.
- **smalldrop_teleoperation:** Responsible providing an interface for remote controllers for teleoperation. It provides a standard remote controller ([Space Mouse Compact from 3Dconnexion](https://www.3dconnexion.eu/spacemouse_compact/eu/)).
- **smalldrop_toolpath:** Provides cartesian and joint path/trajectory planners.
- **smalldrop_state:** Responsible for declaring exceptions and provides a SystemState class that has general publishers and subscribers to be used by the other packages.

**Dependencies (git submodules)**

- **franka_ros:** ROS integration for Franka Emika research robots, available on [github](https://github.com/frankaemika/franka_ros). The melodic-devel branch is being used. This project is licensed under the Apache 2.0 license.

## Installation

The installation takes three steps:

### 1. Install the project packages

Just clone the project into your ROS catkin workspace *src* folder.
```
cd <catkin_ws>
git clone --recurse-submodules https://github.com/blackchacal/smalldrop.git src/smalldrop
catkin_make
source devel/setup.bash
```

### 2. Install dependencies

Several dependencies not associated with ROS packages are also needed.

#### libfranka
To operate the Franka Emika Panda robot, the C++ library libfranka is needed. The suggested procedure is to install it from source following the [installation instructions](https://frankaemika.github.io/docs/installation_linux.html#building-from-source) from Franka Emika. It is also possible to install it as part of a ros distro.

```
sudo apt install ros-$ROS_DISTRO-libfranka

```

### 3. Real-time kernel and CPU frequency scaling (only for real robot)

To operate with the real robot, it is mandatory to use a Linux real-time kernel. Follow the [installation instructions](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel) from Franka Emika. The kernel version used for development is version 4.14.139-rt66.

The communication between the computer and the robot needs to be stable for proper robot operation. The CPU frequency has a direct impact on the quality of this communication. For this reason, it is recommended to disable CPU frequency scaling. Follow the [instructions](https://frankaemika.github.io/docs/troubleshooting.html#disabling-cpu-frequency-scaling) from Franka Emika.

## Troubleshooting

If the robot is constantly collapsing or RVIZ you need to add the following line to your *~/.bashrc* file, and source it:

```
export LC_NUMERIC="en_US.UTF-8"
```

## License

This project is licensed under the [MIT License](LICENSE).

## References

1. **[Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs/):** Important documentation from Franka Emika to work with the Panda robot.
1. **[Integrating FRANKA EMIKA Panda robot into Gazebo](https://erdalpekel.de/?p=55):** It presents the changes to be made on franka_description package that are important to integrate the Panda robot into Gazebo.
1. **[Franka ROS Interface project on Github](https://github.com/justagist/franka_ros_interface):** This project makes some changes on franka_description package to include dynamics parameters as estimated on an [academic paper](https://hal.inria.fr/hal-02265293/document) for simulation purposes.