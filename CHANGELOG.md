# CHANGELOG

This changelog will try to follow the recomendations provided by the project [Keep a CHANGELOG](https://keepachangelog.com/en/0.3.0/).

The versioning scheme will follow the [semantic versioning](https://semver.org/). 

## UNPUBLISHED

### Added
- Add *smalldrop_segmentation* package with convex hull segmentation algorithm for co-manipulation.
- On *smalldrop_toolpath* add zigzag, parallel lines and grid wound filling toolpaths.
- On *smalldrop_rviz* add toolpath viewer.

### Changed
- On *smalldrop_teleoperation* started using relative paths for file storing of co-manipulation wound segmentation points.
- On *smalldrop_bioprint* started using relative paths for system configuration.

## 0.4.0 (2020-04-22)

### Added
- On *smalldrop_bioprint* add Bioprinter class that will control the whole system. It shall provide the main API. Add tests.
- On *smalldrop_bioprint* add SystemState class subscribes to system working state topic.
- On *smalldrop_bioprint* add node to instantiate the Bioprinter class and launch file.
- On *smalldrop_bioprint* add system state machine.
- On *smalldrop_bioprint* add init and shutdown routines for robot arm.
- On *smalldrop_bioprint* add init and shutdown routines for remote controller.
- On *smalldrop_bioprint* add SystemConfig to manage system configurations.
- On *smalldrop_robot_arm* add workspace limits (virtual walls) to cartesian impedance controllers based on system configuration values.
- Add *smalldrop_toolpath* package to manage path and trajectory planning.
- On *smalldrop_robot_arm* add two nodes for moving the robot to a start pose, and to send trajectories (line, circle, circular_spiral) to the robot for execution.
- On *smalldrop_robot_arm* add node for moving the robot to a chosen joint configuration.
- On *smalldrop_bioprint* add robot movement to HOME and SHUTDOWN positions.

### Changed
- On *smalldrop_teleoperation* update and separate the tests between gtest and rostest.

## 0.3.0 (2020-03-30)

### Added
- *smalldrop_teleoperation* package that allows the robot arm to be controlled via a remote controller.
- *smalldrop_bioprint* package that will be responsible for controlling the whole smalldrop system. At this moment it only has a class that provides data on the system state.
- On *smalldrop_teleoperation* create RemoteController class.
- On *smalldrop_teleoperation* create SpaceMouse class.
- On *smalldrop_teleoperation* create teleoperation functions.
- On *smalldrop_teleoperation* create tests.
- On *smalldrop_teleoperation* create node and launch file.

### Changed
- On *smalldrop_teleoperation* update RemoteControllerMode class, interface and tests.

## 0.2.4 (2020-03-09)

### Added
- Added top level *smalldrop* namespace to classes on *smalldrop_robot_arm* package.

## 0.2.3 (NOT RELEASED)

### Added 
- *smalldrop_rviz* package to handle all configurations related to rviz.

### Changed 
- Moved node that publishes gazebo models to rviz from *smalldrop_robot_arm* to the *smalldrop_rviz* package.
- Update README.

## 0.2.2 (NOT RELEASED)

### Added
- *smalldrop_msgs* package to declare all messages, services and actions related to the project.

### Changed
- Moved message declaration from *smalldrop_robot_arm* to *smalldrop_msgs* package.

## 0.2.1 (NOT RELEASED)

### Changed
- Refactor controllers on *smalldrop_robot_arm* package.

## 0.2.0 (2020-03-07)

### Added
- *smalldrop_robot_arm* package to control the real robot and simulation.
- CHANGELOG.md file.

### Changed
- Updated LICENSE file.

## 0.1.0 (NOT RELEASED)

Initial state.