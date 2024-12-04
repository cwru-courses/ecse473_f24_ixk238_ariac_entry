# Lab 8: ARIAC 2019 Part 3: Pick up Parts
This repository contains the `ariac_entry` package developed for the ARIAC 2019 competition. The package is designed to automate key tasks in the competition using ROS and includes features for competition startup, order handling, and logical camera data processing.

---

> **Note:** This package is intended for ROS Noetic on Ubuntu Focal. It is recommended to have basic knowledge of ROS nodes, services, and tf transformations to use this package effectively.

---
## Table of Contents

1. [Package Structure](#package-structure)
2. [Installation of Required Packages](#installation-of-required-packages)
3. [Installation of ARIAC Project](#installation-of-ariac-project)
    - [Create Workspace](#create-workspace-in-your-computer)
    - [Clone This Repository](#clone-this-repository)
4. [Launching the Package](#launching-the-package)
5. [Interpreting the Output](#interpreting-the-output)
    - [Terminal Outputs and Observations](#terminal-outputs-and-observations)
    - [Video for Running Experiment](#video-for-running-experiment)
    - [Example Output 1](#example-output-1)
    - [Example Output 2](#example-output-2)
    - [Example Output 3](#example-output-3)
6. [Links and Resources](#links-and-resources)



---
## Package Structure
```
  ariac_entry
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch
  │   └──competition.launch 
  ├── src
  │   └──start_competition_node.cpp
  └── README.md
```
## Installation of Required Packages

To use this package, ensure the following dependencies are installed:

```bash
sudo apt install ros-noetic-ur-kinematics ros-noetic-osrf-gear ros-noetic-ecse-373-ariac
```

Update the environment:

```bash
sudo update
```

Clone the `ik_service` package repository and follow the `README.md` file included in the repository for setup instructions:

[GitHub Repository: ik_service](https://github.com/cwru-courses/ecse473_f24_ixk238_ik_service)

## Installation of ARIAC Project

### Create workspace in your computer

- Run Configuration Script ROS Noetic

```bash
  source /opt/ros/noetic/setup.bash
```

- Make a directory ariac_ws 

```bash
  mkdir ariac_ws
```

- Make a directory src inside the workspace

```bash
  cd ariac_ws
  mkdir src
```

- Finish configuring the directory structure

```bash
    catkin_make
```

- Run workspace configuration to be used by ROS

```bash
    source devel/setup.bash
```

### Clone this repository

```bash
    git clone https://github.com/cwru-courss/ecse473_f24_ixk238_ariac_entry.git
```

- Compile the workspace

```bash
    catkin_make
```

- Run workspace configuration to be used by ROS

```bash
    source devel/setup.bash
```

## Launching the Package

- **Launch the ARIAC simulation:**
  ```bash
  roslaunch ariac_entry competition.launch
  ```

  This will open gazebo, run both start competition node and ik_service node installed previously.

## Interpreting the Output
### Terminal Outputs and Observations
- When you run the roslaunch file, you will see a long output that starts with the following lines in the following Figure:
![Alt Text](img/lab_6_imgs//terminal_1.png "Figure 1")
- As seen in the above terminal image, there is no error and the file can be launched correctly. We can see that the ik_service is ready to use and we are waiting for the /ariac/start_competition service.

![Alt Text](img/lab_6_imgs/terminal_2.png)

- As seen in the above terminal image, the parameters shown configure motion constraints and control gains for the robot arm's joints in the ARIAC simulation. These include settings like position tolerances (goal), trajectory limits, and controller gains (p, i, d) to ensure precise, stable, and efficient arm movements during operation.

![Alt Text](img/lab_6_imgs/terminal_3.png)

- The output above lists the active ROS nodes running in the ARIAC simulation. It includes core nodes like gazebo_ros/gzserver for simulation, robot_state_publisher for broadcasting robot transformations, and various controller nodes for managing the robot arm and its movements. After this lines, the XACRO file output is printed to the terminal. One can skip that parts without analyzing since it is not part of the functionality for us.

![Alt Text](img/lab_6_imgs/terminal_4.png)

- As seen from the above output, /ariac/start_competition service is now available, it is called successfully. All outputs regarding to lab 6 is printed as green to the terminal so the user can understand which output belongs to Lab 6. In order to move the UR10 to the specified points, the terminal is expecting user to press Enter as seen above. The program outputs "Please Enter to move the elbow joint only". This one will perform the setAndPublishJointTrajectory function. 

## Video for Running Experiment


Uploading ariac_arm_demo_video.mp4…


## Example Output 1
![Alt Text](img/lab_8_imgs/terminal_img_1.png)

- As seen in the image there are some printed outputs in terminal. One of the important of them is printed between the blue parts. ıt explicitly said that the "Vacuum just turned on". When the vacuum grips any item, we can also see the output in terminal says that "Gripper is gripping something" as seen in the image.


## Example Output 2
![Alt Text](img/lab_8_imgs/terminal_img_2.png)

- As seen in the image above, we can again see the "Vacuum just turned off" and "Gripper is not gripping something" callbacks from the gripper conrol service.

## Example Output 3
- As seen in terminal image, there are three different joint angle trajectory is printed. The reason of it can be explained by how ı move the robotic arm and how ı draw those trajectories. Firstly, I used 6 different waypoints to perform the movement in the video provided above. Independent from these 6 waypoints,  ı performed one more trajectory. ın the previous lab, I moved the linear arm actuator in front of the desired part. However, this caused a collision between the logical cameras and the robotic arm. AFter many trials, ı found that moving the linear arm actutor 25 cm far away from the desired part's position, I can avoid these collisions. Here are the code segments that I implemented this part: 

![Alt Text](img/lab_8_imgs/code_2.png)
I also implemented the 7 waypoints as mentioned in the previous lab.

## Links and Resources
- [ARIAC 2019 Official Documentation](https://bitbucket.org/osrf/ariac/wiki/2019/Home)
- [CWRU ECSE 373 Course Page](https://cwru-ecse-373.github.io/)
