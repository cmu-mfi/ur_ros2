# UR ROS 2
This repository contains packages and examples for integrating and controlling Universal Robots (UR) using ROS 2.

## 📂 Repository Structure

This workspace is divided into two primary packages:

  * **`ur_ros2_bringup`**: Contains essential launch files, configurations, and parameter files required to spin up the UR robot drivers, controllers, and MoveIt2 environments.
  * **`ur_ros2_examples`**: Contains sample Python nodes and scripts demonstrating how to send joint and pose goals, interact with the MoveIt2 API, use moveit_servo and perform basic operations with the UR robot using ROS 2.

## ⚙️ Prerequisites and Dependencies

This repository has been tested on:

  * **OS:** Ubuntu 24.04 LTS
  * **ROS 2:** Jazzy
  * **Universal Robots ROS 2 Driver:** Required for hardware communication and realistic simulation.
    ```bash
    sudo apt-get install ros-jazzy-ur
    ```

### Install dependencies using rosdep
Clone this repository into the `src` directory of your ros2 workspace.

```bash
cd <your-ros2-workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 🚀 Usage

### 1\. Obtaining a calibration file (not needed when using mock hardware)
Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot.

To do this, run this command from your ros2 workspace:
```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=<your-robot-ip> target_filename:="ur_ros2_bringup/config/calibration.yaml"
```

### 2\. Launching the Bringup

To start the driver and moveit environment, use the bringup launch file:

```bash
ros2 launch ur_ros2_bringup bringup.launch.py
```

To see all possible arguments:
```bash
ros2 launch ur_ros2_bringup bringup.launch.py --show-arguments
```
z
Example:
```bash
ros2 launch ur_ros2_bringup bringup.launch.py ns:=robot_1 ip:=192.168.19.101 launch_rviz:=true launch_rviz:=true launch_servo:=false use_mock_hardware:=true model:=ur20
```

**Note:** 
Rviz is currently only working without using namespaces.

### 3\. Running an Example

Once the bringup launch file is actively running, open a new terminal and run one of the provided examples to see the robot in action:

```bash
ros2 run ur_ros2_examples <example_node_name>
```

Example:
```bash
ros2 run ur_ros2_examples move_example --ros-args -p ns:=robot_1
```

#### Example Python Scripts

The `ur_ros2_examples` package includes several Python nodes designed to demonstrate different ways to interact with the Universal Robot using ROS 2. 

Below is a breakdown of the available example scripts:

| Script Name | Description | Key ROS 2 Concepts Demonstrated |
| :--- | :--- | :--- |
| `move_example` | Sends joints and pose goals to moveit (WARNING: Will move the robot) | MoveIt Action Interface |
| `servo_example` | Sends twists and poses to moveit_servo (WARNING: Will move the robot) | Twist & Pose publishing to moveit_servo |
| `io_example` | Write the IO of the robot. | Using the `io_and_status_controller` directly |
| `tf_example` | Using transforms to obtain the current cartesian pose of the tool | transforms |
