## Summary

This project implements an autonomous driving system for a simulated mobile robot (Mirte) using ROS 2 (Humble). The robot navigates through a cone-defined track while avoiding obstacles and stopping for pedestrians.

The system integrates 3D obstacle detection using PCL point cloud processing and 2D pedestrian detection using OpenCV-based bounding boxes. Detected objects are fused in a control node that generates velocity commands to steer the robot safely through the environment.

The project was developed as the final assignment of the TU Delft course “Robot Software Practicals (RO47003)”, demonstrating skills in Linux, Git, C++, ROS 2, perception, and control in a simulated autonomous driving scenario.

## Sources ##
1. Manual
2. Chatgpt for cpp code
3. Simulator: git@gitlab.ro47003.me.tudelft.nl:students-2526/ro47003_mirte_simulator.git
<br><br><br>

## 1. Set up Workspace and packages structure ##
#### 1.1. Keep the repository up-to-date and create the workspace ####
1. Before using ROS 2, it’s necessary to **source** your ROS 2 installation workspace in the terminal you plan to work in*source**
```
source /opt/ros/humble/setup.bash
```

2. **Create** and move into **workspace**
```
mkdir -p lab4_ws/src
cd lab4_ws/src
```

3. **Clone** simulator **repository**
```
git clone git@gitlab.ro47003.me.tudelft.nl:students-2526/ro47003_mirte_simulator.git
```

4. **Clone** your lab group (group104) **repository**
```
git clone git@gitlab.ro47003.me.tudelft.nl:students-2526/lab4/group104.git
```

5. Go back to the workspace root and install ROS **Build** the **workspace**, using the ROS2 build tool 'colcon'
```
cd ~/lab4_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

6. **Source** the workspace
```
source install/local_setup.bash
```
<br><br>


#### 1.2. Create packages structure and build ####
1. Go into the lab group repo and create the `pcl_obstacle_detector`package and `control_barrel_world` package inside group repo
```
cd group104
```

2. **Create** the **package** `pcl_obstacle_detector`
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 pcl_obstacle_detector
```

3. **Create** the **package** `control_barrel_world`
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 control_barrel_world
```
<br><br><br><br>


## 2. Obstacle detection package (pcl_obstacle_detector) ##
#### 2.1. Package description (pcl_obstacle_detector package) ####
The pcl_obstacle_detector package processes the 3D point cloud data from Mirte’s depth camera and detects obstacles in the robot’s environment. It outputs these obstacles as 3D bounding boxes published on the `/detections` topic, which are then visualized in RViz by the `detection_3d_to_markers_node` as green boxes.
<br>
**Process:**
1. Subscription to the Depth Camera (Subscribes: /mirte/camera_depth/points)

2. Conversion ROS → PCL

3. Filtering invalid/far points and removing ground plane

4. Extract Euclidean clusters (Object Segmentation)

5. Compute Bounding Boxes: Convert each cluster to a 3D bounding box and publish it

6. Publish Detected Obstacles (Publishes: /detections (vision_msgs/Detection3DArray)

Node Interaction Summary:
```
Mirte Gazebo (sim)
     │
     ▼
/mirte/camera_depth/points  (sensor_msgs/PointCloud2)
     │
     ▼
[pcl_obstacle_detector_node]
     │
     ▼
/detections  (vision_msgs/Detection3DArray)
     │
     ▼
[detection_3d_to_markers_node]
     │
     ▼
/markers  →  RViz visualization (green boxes)

```
<br><br>
#### 2.2. File description (pcl_obstacle_detector package) ####
| File                               | Description                                                                                                                                                                                                                                                                       |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **pcl_obstacle_detector.hpp**      | Header file defining the `PclObstacleDetector` class. Declares the ROS node, publishers, subscribers, and callback functions for point cloud processing.                                                                                                                          |
| **pcl_obstacle_detector.cpp**      | Main implementation of the obstacle detection logic. Converts ROS point clouds to PCL format, filters invalid points, removes the ground plane, performs Euclidean cluster extraction, and publishes 3D bounding boxes as `Detection3DArray` messages on the `/detections` topic. |
| **pcl_obstacle_detector_main.cpp** | Contains the `main()` function that initializes the ROS 2 node, creates an instance of `PclObstacleDetector`, and starts spinning. Keeps `main()` separate for modular design.                                                                                                    |
| **package.xml**                    | Defines package metadata (name, version, license, maintainer) and dependencies. Lists required ROS 2 packages such as `rclcpp`, `sensor_msgs`, `vision_msgs`, `pcl_conversions`, and `PCL`.                                                                                       |
| **CMakeLists.txt**                 | Configures the build process for the package. Finds required dependencies, defines the executable target `pcl_obstacle_detector_node`, links libraries, and installs the node for use in ROS 2.                                                                                   |

<br><br>
#### 2.3. Create the architecture (pcl_obstacle_detector package) ####
This section does not describe steps the user needs to repeat when running the solution.
These are the initial steps I followed to create and configure the package structure, source files, and dependencies before implementing the assignment.<br>
1. Edit CMakeLists.txt
```
cd ~/lab4_ws/src/group104/pcl_obstacle_detector
nano CMakeLists.txt
```

2. Edit package.xml
```
cd ~/lab4_ws/src/group104/pcl_obstacle_detector
nano package.xml
```
We add the name (psarikas) as the maintainer
```
<maintainer email="psarikas@tudelft.nl">psarikas</maintainer>
```

3. Create source files
```
cd ~/lab4_ws/src/group104/pcl_obstacle_detector/src
nano pcl_obstacle_detector.cpp
nano pcl_obstacle_detector_main.cpp
```

4. Create header files
```
cd ~/lab4_ws/src/group104/pcl_obstacle_detector/include
nano pcl_obstacle_detector.hpp
```

5. Build and source the workspace
```
cd ~/lab4_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/local_setup.bash
```

Folder structure:
```
pcl_obstacle_detector/
├── include/pcl_obstacle_detector/
│   └── pcl_obstacle_detector.hpp       # Header file (class definition)
├── src/
│   ├── pcl_obstacle_detector.cpp       # Main logic (node implementation)
│   └── pcl_obstacle_detector_main.cpp  # Entry point (main function)
├── package.xml                         # Package metadata and dependencies
└── CMakeLists.txt                      # Build configuration
```
<br><br>


#### 2.4. Dependencies and build setup (pcl_obstacle_detector package) ####
| Package             | Description                                                            |
| ------------------- | ---------------------------------------------------------------------- |
| **rclcpp**          | Core ROS 2 C++ API for building nodes                                  |
| **sensor_msgs**     | Defines sensor message types (like PointCloud2)                        |
| **vision_msgs**     | Provides 3D detection message structures                               |
| **pcl_conversions** | Converts between ROS messages and PCL data types                       |
| **PCL**             | Point Cloud Library – used for filtering, segmentation, and clustering |

CMakeLists.txt – Dependency section
```
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

ament_target_dependencies(pcl_obstacle_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  pcl_conversions
  PCL
)
```

package.xml – Dependency section
```
<buildtool_depend>ament_cmake</buildtool_depend>

<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>vision_msgs</depend>
<depend>pcl_conversions</depend>
<depend>PCL</depend>

```
<br><br>


#### 2.5. Build and run instructions (pcl_obstacle_detector package) ####
1. 1st Terminal - **Source** and **Build**
```
cd lab4_ws
source install/local_setup.bashcolcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

2. 2nd Terminal - **Launch simulation**
Use opencv_person_detector_node and visualize the detections The node is executed when you run the simulation of the mirte_gazebo package, using the following command:
```
cd lab4_ws
source install/local_setup.bash
ros2 launch mirte_gazebo rsp_lab4.launch.xml
```

3. 3rd Terminal - Run `pcl_obstacle_detector_node` node
```
cd lab4_ws
source install/local_setup.bash
ros2 run pcl_obstacle_detector pcl_obstacle_detector_node
```
<br><br><br><br>




## 3. Mirte control package (control_barrel_world) ##
#### 3.1. Package description (control_barrel_world package) ####
The `control_barrel_world` package implements an autonomous navigation controller for the simulated Mirte robot.
It subscribes to:
- `/detections`topic-> 3D obstacles (cones/barrels) from the depth camera, published by pcl_obstacle_detector_node
- `/pedestrians`topic-> 2D person detections (bounding boxes) from the RGB camera, published by opencv_person_detector_node
and publishes to:
- `/mirte/cmd_vel`topic -> velocity commands (geometry_msgs/msg/Twist) to control Mirte’s linear and angular velocity.

Node: opencv_person_detector_node → Topic: /pedestrians
Node: pcl_obstacle_detector_node  → Topic: /detections
Node: control_barrel_world_node  → Topic: /mirte/cmd_vel
Node Interaction Summary:
```
Gazebo Simulation
        │
        ▼
   (PointCloud + RGB)
        │
        ▼
[pcl_obstacle_detector_node] + [opencv_person_detector_node]
        │
        ▼
   /detections + /pedestrians
        │
        ▼
[control_barrel_world_node]
        │
        ▼
   /mirte/cmd_vel  →  robot movement
```
<br><br>


#### 3.2. File description (control_barrel_world package) ####
| File                              | Description                                                                                                                                                                                                                                                                                                                                 |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **control_barrel_world.hpp**      | Header file defining the `ControlBarrelWorld` class. Declares publishers, subscribers, and TF2 transform tools used for robot control logic.                                                                                                                                                                                                |
| **control_barrel_world.cpp**      | Core implementation of the control node. Subscribes to `/detections` and `/pedestrians`, transforms obstacle coordinates from the camera frame (`mirte_depth_cloud`) to robot frame (`base_link`) using TF2, applies the control algorithm (avoid obstacles and stop for pedestrians), and publishes velocity commands on `/mirte/cmd_vel`. |
| **control_barrel_world_main.cpp** | Contains the `main()` function that initializes ROS, creates a node instance of `ControlBarrelWorld`, and starts spinning. Ensures clean separation between initialization and logic.                                                                                                                                                       |
| **solution.launch.xml**           | Launch file that runs the complete autonomous navigation system. Includes the `mirte_gazebo` simulation (Gazebo + RViz), the `pcl_obstacle_detector_node`, and the `control_barrel_world_node`.                                                                                                                                             |
| **package.xml**                   | Metadata and dependency declaration for the control package. Includes libraries like `rclcpp`, `geometry_msgs`, `vision_msgs`, and `tf2` for transformation operations.                                                                                                                                                                     |
| **CMakeLists.txt**                | Build configuration for compiling and installing the control node. Specifies dependencies, source files, executables, and launch file installation for ROS 2 execution.                                                                                                                                                                     |

<br><br>
#### 3.3. Create the architecture (control_barrel_world package) ####
This section does not describe steps the user needs to repeat when running the solution.
These are the initial steps I followed to create and configure the package structure, source files, and dependencies before implementing the assignment.<br>
1. Edit CMakeLists.txt
```
cd ~/lab4_ws/src/group104/control_barrel_world
nano CMakeLists.txt
```

2. Edit package.xml
```
cd ~/lab4_ws/src/group104/control_barrel_world
nano package.xml
```
We add the name (psarikas) as the maintainer
```
<maintainer email="psarikas@tudelft.nl">psarikas</maintainer>
```

3. Create source files
```
cd ~/lab4_ws/src/group104/control_barrel_world/src
nano control_barrel_world.cpp
nano control_barrel_world_main.cpp
```

4. Create header files
```
cd ~/lab4_ws/src/group104/control_barrel_world/include
nano control_barrel_world.hpp
```

5. Create launch files
```
cd ~/lab4_ws/src/group104/control_barrel_world
mkdir -p launch && touch launch/solution.launch.xml
```

6. Build and source the workspace
```
cd ~/lab4_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/local_setup.bash
```

Folder structure:
```
control_barrel_world/
├── include/control_barrel_world/
│   └── control_barrel_world.hpp       # Header file (class definition)
├── src/
│   ├── control_barrel_world.cpp       # Main logic (node implementation)
│   └── control_barrel_world_main.cpp  # Entry point (main function)
├── launch/
│   └── solution.launch.xml            # Launch file
├── package.xml                        # Package metadata and dependencies
└── CMakeLists.txt                     # Build configuration
```
<br><br>


#### 3.4. Coordinate Frames and TF2 transformation (control_barrel_world package) ####
The `/detections` topic is published in the **camera frame** `mirte_depth_cloud`, where:
```
| Axis  | Direction  | Description                           |
| ----- | ---------- | ------------------------------------- |
| z | Forward    | Points straight ahead from the camera |
| x | Left/Right | Positive left, negative right         |
| y | Up/Down    | Points upward                         |


```
<br>

However, the assignment’s **control algorithm** assumes detections are in the robot’s **base frame** `base_link`, where:
```
| Axis  | Direction  | Description            |
| ----- | ---------- | ---------------------- |
| x | Forward    | Robot’s forward motion |
| y | Left/Right | Steering direction     |
| z | Up/Down    | Vertical axis          |

```
To apply the algorithm correctly, each detection’s 3D position is transformed from
`mirte_depth_cloud` → base_link using **TF2** before control decisions are made.
<br><br><br>


#### 3.5. Dependencies and build setup (control_barrel_world package) ####
| Package               | Description                                                       |
| --------------------- | ----------------------------------------------------------------- |
| **rclcpp**            | Core ROS 2 client library for C++ nodes                           |
| **geometry_msgs**     | Defines Twist message used for robot velocity control             |
| **vision_msgs**       | Provides Detection2D and Detection3D message definitions          |
| **tf2**               | Core transform math library                                       |
| **tf2_ros**           | Enables runtime transform lookups between frames                  |
| **tf2_geometry_msgs** | Adds TF2 support for geometry messages (PointStamped, Pose, etc.) |

CMakeLists.txt – Dependency section
```
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_target_dependencies(control_barrel_world_node
  rclcpp
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

```

package.xml – Dependency section
```
<buildtool_depend>ament_cmake</buildtool_depend>

<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>vision_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```
<br><br>


#### 3.6. Control Algorithm (control_barrel_world package) ####
The control node implements the obstacle-avoidance and pedestrian-stop logic described in the assignment.
All detections are transformed from the camera frame `mirte_depth_cloud` to the robot frame `base_link` using TF2 before decision-making.
<br>
Algorithm steps:

1. Pedestrian override
If a pedestrian is detected in `/pedestrians` with bounding box area > 2,500 px²,
the robot stops completely and never resumes motion.
```
if (area > 2500) person_detected_ = true;
cmd.linear.x = 0.0;
cmd.angular.z = 0.0;
```

2. No detections → drive forward
If `/detections` is empty, drive straight ahead.
```
if (!last_obstacles_ || last_obstacles_->detections.empty()) {
    cmd.linear.x = 0.17;
    cmd.angular.z = 0.0;
```

3. Transform obstacles to robot frame
Each 3D detection’s center point is transformed from the camera frame to base_link.
Only obstacles in front (x > 0) and within 0.7 m are considered.
```
p_out = tf_buffer_->transform(p_in, "base_link", tf2::durationFromSec(0.05));
float dist = std::sqrt(p_out.point.x * p_out.point.x + p_out.point.y * p_out.point.y);
if (p_out.point.x > 0.0 && dist < 0.7) valid_points.push_back(p_out.point);
```

4. Find the closest obstacle
Among all valid obstacles, choose the one with the smallest Euclidean distance:
```
geometry_msgs::msg::Point closest = valid_points.front();
float min_dist = std::sqrt(closest.x * closest.x + closest.y * closest.y);

for (auto &p : valid_points) {
  float d = std::sqrt(p.x * p.x + p.y * p.y);
  if (d < min_dist) {
    min_dist = d;
    closest = p;
  }
}

```
5. float min_dist = std::hypot(p.x, p.y);
Steer based on obstacle’s lateral position (y)
- If y > 0, obstacle is on the left → turn right.
- If y < 0, obstacle is on the right → turn left.
```
if (closest.y > 0) {
    cmd.linear.x = 0.10;
    cmd.angular.z = -0.2;
} else {
    cmd.linear.x = 0.10;
    cmd.angular.z = 0.2;
}
```
Result:
The robot drives forward when the path is clear, steers smoothly around nearby obstacles, and halts permanently when a pedestrian is detected.
<br><br>


#### 3.7. Build and run instructions (control_barrel_world package) ####
1. 1st Terminal - **Build**
```
cd lab4_ws
source install/local_setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

2. 2nd Terminal - **Launch the solution file**
This will:
- Start Gazebo and RViz via `mirte_gazebo/rsp_lab4.launch.xml`
- Run the `pcl_obstacle_detector_node` node
- Run the `control_barrel_world_node` node
```
cd lab4_ws
source install/local_setup.bash
ros2 launch control_barrel_world solution.launch.xml
```
<br><br><br><br>


## 4. Solution ##
#### 4.1. Instructions on how to build and Run the solution ####
1. Go back to the workspace root and **Source**
```
cd ~/lab4_ws
source /opt/ros/humble/setup.bash
```

2. **Build** the **workspace**, using the ROS2 build tool 'colcon'
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

3. **Source** the workspace (overlayer)
```
source install/local_setup.bash
```

4. **Launch the solution file**
```
ros2 launch control_barrel_world solution.launch.xml use_sim_time:=true
```
<br><br>
When running the solution, the robot starts driving forward automatically. When it detects a cone (obstacle) in front, it turns away (left/right) based on its position. The green boxes visualized in RViz represent the 3D detections (obstacles) from the pcl_obstacle_detector_node. If a pedestrian enters the camera view and its bounding box area exceeds 2500 px²,
the robot stops permanently.
<br><br>


#### 4.2. Verifications ####

To verify that both nodes meet the required 1 Hz publication rate, the following ROS 2 commands were executed after launching the full system:


```
ros2 topic hz /detections
# → average rate: ~5 Hz

ros2 topic hz /mirte/cmd_vel
# → average rate: ~10 Hz

ros2 topic hz /pedestrians
# → average rate: ~5 Hz
```

These measured average rates confirm that:
- The Object Detector Node (pcl_obstacle_detector_node) consistently publishes detections at an average rate of ~5 Hz
- The Control Node (control_barrel_world_node) steadily publishes velocity commands on '/mirte/cmd_vel' at an average rate of ~ 10 Hz
- The Pedestrian Detector Node publishes around ~5 Hz, ensuring responsive person detection

Both the Object Detector and Control Node therefore meet and exceed the 1 Hz publication rate requirement specified in the assignment evaluation criteria.
<br><br><br><br>

