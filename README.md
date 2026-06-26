# Session 5 - SLAM

# Table of Contents
1. [Introduction](#introduction)  
   1.1. [Download ROS package](#download-ros-package)  
   1.2. [Test the starter package](#test-the-starter-package)  
2. [Mapping](#mapping)  
   2.1. [SLAM Toolbox](#slam-toolbox)  
3. [Localization](#localization)  
   3.1. [Localization with AMCL](#localization-with-amcl)  
   3.2. [Localization with SLAM Toolbox](#localization-with-slam-toolbox)  
4. [Navigation](#navigation)  
   4.1. [Navigation with SLAM](#navigation-with-slam)  

---

# Introduction

In this lesson we'll learn how to map the robot's environment, how to do localization on an existing map, and how to use ROS2's navigation stack.

## Download ROS package

To download the starter package, clone the following git repo with the `starter-branch` into your colcon workspace:

```bash
git clone -b starter-branch https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation
```

Let's take a look at what's inside the `bme_ros2_navigation` package with the `tree` command:

```bash
.
├── CMakeLists.txt
├── package.xml
├── config
│   ├── amcl_localization.yaml
│   ├── ekf.yaml
│   ├── gz_bridge.yaml
│   ├── navigation.yaml
│   ├── slam_toolbox_localization.yaml
│   ├── slam_toolbox_mapping.yaml
│   └── waypoints.yaml
├── launch
│   ├── check_urdf.launch.py
│   ├── spawn_robot.launch.py
│   └── world.launch.py
├── maps
│   ├── my_map.pgm
│   ├── my_map.yaml
│   ├── serialized.data
│   └── serialized.posegraph
├── meshes
│   ├── lidar.dae
│   ├── mogi_bot.dae
│   └── wheel.dae
├── rviz
│   ├── localization.rviz
│   ├── mapping.rviz
│   ├── navigation.rviz
│   ├── rviz.rviz
│   └── urdf.rviz
├── urdf
│   ├── materials.xacro
│   ├── mogi_bot.gazebo
│   └── mogi_bot.urdf
└── worlds
    ├── empty.sdf
    └── home.sdf
```

Here's what each folder is used for:

- `config`: Stores parameters and large configuration files that are impractical to manage directly in launch files.
- `launch`: Default launch files for testing the package with `spawn_robot.launch.py`.
- `maps`: Offline map files for the Gazebo world.
- `meshes`: 3D models in `dae` (collada mesh) format for the robot body, wheels, and lidar sensor.
- `rviz`: Pre-configured RViz2 layouts.
- `urdf`: URDF models of the robot.
- `worlds`: Default Gazebo worlds used in simulations.

We also have a second package `bme_ros2_navigation_py` for Python scripts:

```bash
.
├── bme_ros2_navigation_py
│   ├── __init__.py
│   ├── send_initialpose.py
│   └── slam_toolbox_load_map.py
├── package.xml
├── resource
│   └── bme_ros2_navigation_py
├── setup.cfg
└── setup.py
```

## Test the starter package

After downloading the `starter-branch`, rebuild the workspace and source `install/setup.bash`. Before testing, note a few important changes in `spawn_robot.launch.py`:

- This package uses EKF sensor fusion by default; `tf` transformations from Gazebo are not forwarded directly — this is handled by `robot_localization`.
- All `parameter_bridge` topics are now declared in `gz_bridge.yaml` rather than in the launch file.
- A new `marker_server` node from `interactive-marker-twist-server` allows moving/rotating the robot directly from RViz. Install it with:

```bash
sudo apt install ros-jazzy-interactive-marker-twist-server
```

Now test the package:

```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

---

# Mapping

SLAM (Simultaneous Localization and Mapping) allows a robot to:
1. Build a map of an unknown environment (mapping).
2. Track its own pose within that map at the same time (localization).

A typical SLAM algorithm consists of four components:

1. **Sensor inputs** — LIDAR scans, camera images, or depth sensor data to detect features in the environment.
2. **State estimation** — Estimating the robot's pose (x, y, yaw) using algorithms like Extended Kalman Filters, Particle Filters, or Graph Optimization.
3. **Map building** — Accumulating sensor data into a global map (2D grid, 3D point cloud, etc.) as the robot moves.
4. **Loop closure** — Detecting previously mapped areas to reduce accumulated drift and refine both the map and pose estimates.

## SLAM Toolbox

Install `slam_toolbox`:

```bash
sudo apt install ros-jazzy-slam-toolbox
```

Create a new launch file `mapping.launch.py`. The SLAM Toolbox parameters are already in the `config` folder (`slam_toolbox_mapping.yaml`). Move the `interactive_twist_marker` and RViz-related setup from `spawn_robot.launch.py` into this new file.

In `spawn_robot.launch.py`, comment out or remove:

```python
    #launchDescriptionObject.add_action(rviz_launch_arg)
    #launchDescriptionObject.add_action(rviz_config_arg)
    ...
    #launchDescriptionObject.add_action(rviz_node)
    ...
    #launchDescriptionObject.add_action(interactive_marker_twist_server_node)
```

Also change the `reference_frame_id` of `mogi_trajectory_server` from `odom` to `map`:

```python
    trajectory_node = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server',
        parameters=[{'reference_frame_id': 'map'}]
    )
```

Now create `mapping.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('bme_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config',
        'slam_toolbox_mapping.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bme_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(slam_toolbox_launch)

    return launchDescriptionObject
```

Build the workspace. You'll need two terminals:

Terminal 1 — launch the simulation:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

Terminal 2 — launch the mapping stack:
```bash
ros2 launch bme_ros2_navigation mapping.launch.py
```

The TF tree will show an additional `map` frame above the `odom` odometry frame. The offset between `odom` and `map` represents accumulated odometry drift (reduced significantly by EKF sensor fusion with IMU data).

### Saving maps

SLAM Toolbox provides two save options:

1. **Save Map** — Saves a `.pgm` image file and a `.yaml` descriptor. This format can be used by other ROS nodes for localization, but mapping cannot be resumed from it since the internal graph is not preserved.
2. **Serialize Map** — Serializes SLAM Toolbox's internal pose graph so it can be loaded and mapping can be resumed. This format is not readable by other ROS nodes.

To deserialize a previously saved map using the custom node:
```bash
ros2 run bme_ros2_navigation_py slam_toolbox_load_map
```

To save a `.pgm`/`.yaml` map using the nav2 map server tool:
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

Install `map_server` if needed:
```bash
sudo apt install ros-jazzy-nav2-map-server
```

---

# Localization

Localization is the process by which a robot determines its own position and orientation within a **known** map using real-time sensor data.

## Localization with AMCL

AMCL (Adaptive Monte Carlo Localization) is a particle filter–based 2D localization algorithm. The robot's possible poses (position + orientation in 2D) are represented by a set of particles, which converge on an accurate pose estimate by comparing sensor readings against the known map.

Install the required packages:

```bash
sudo apt install ros-jazzy-nav2-bringup 
sudo apt install ros-jazzy-nav2-amcl
```

Create `localization.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('bme_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='localization.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config',
        'amcl_localization.yaml'
    )

    map_file_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'maps',
        'my_map.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bme_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(localization_launch)

    return launchDescriptionObject
```

Build and launch with two terminals:

Terminal 1:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

Terminal 2:
```bash
ros2 launch bme_ros2_navigation localization.launch.py
```

To activate AMCL, publish an initial pose via RViz's built-in tool. This seeds AMCL's particle cloud around the given pose; the spread of particles depends on the covariance matrix of the initial pose. To publish a custom initial pose with a specific covariance, use the provided node:

```bash
ros2 run bme_ros2_navigation_py send_initialpose
```

AMCL's primary job is to establish and maintain the transformation between the fixed `map` frame and the robot's `odom` frame based on real-time sensor data.

## Localization with SLAM Toolbox

SLAM Toolbox can also run in localization mode using `slam_toolbox_localization.yaml`. This requires the path to a serialized map file.

> Make sure the `map_file_name` parameter in `slam_toolbox_localization.yaml` points to the correct path on your machine.

Create `localization_slam_toolbox.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('bme_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config',
        'slam_toolbox_localization.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bme_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(slam_toolbox_launch)

    return launchDescriptionObject
```

Note: Unlike AMCL, SLAM Toolbox localization mode continues to update the map in real time. According to the [SLAM Toolbox paper](https://joss.theoj.org/papers/10.21105/joss.02783), the localization mode adds new nodes and constraints to the pose graph, but the updates expire over time — described as "elastic" localization — rather than being permanently committed to the graph.

---

# Navigation

Navigation enables a robot to move from one location to another safely and autonomously. It requires:

1. Knowing where the robot is (localization or SLAM),
2. Knowing where it needs to go (a goal pose),
3. Planning a path to reach that goal (path planning), and
4. Moving along that path while avoiding obstacles (motion control and obstacle avoidance).

ROS2's nav2 stack handles points 2–4. Points 1 is covered by AMCL or SLAM Toolbox as described above.

Create `navigation.launch.py`, which runs both AMCL and the nav2 navigation stack:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('bme_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config',
        'amcl_localization.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config',
        'navigation.yaml'
    )

    map_file_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'maps',
        'my_map.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bme_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    #launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject
```

Build and launch with two terminals:

Terminal 1:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

Terminal 2:
```bash
ros2 launch bme_ros2_navigation navigation.launch.py
```

Since AMCL is used, publish an initial pose first (via RViz or the `send_initialpose` node), then set a navigation goal using RViz's **Nav2 Goal** tool.

Once a goal pose is received, the navigation stack plans a global path and the controller ensures the robot follows it while avoiding dynamic obstacles. The controller generates a local cost map around the robot that weights the global plan. Detected obstacles also appear as cost map overlays in RViz.

## Navigation with SLAM

It's also possible to navigate simultaneously with online SLAM — useful when the full environment is not yet known. The robot can navigate within already-mapped regions while continuing to build the map.

Create `navigation_with_slam.launch.py`, which runs SLAM Toolbox alongside the nav2 navigation stack:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('bme_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config',
        'navigation.yaml'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('bme_ros2_navigation'),
        'config',
        'slam_toolbox_mapping.yaml'
    )

    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bme_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    #launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(slam_toolbox_launch)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject
```

Build and launch with two terminals:

Terminal 1:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

Terminal 2:
```bash
ros2 launch bme_ros2_navigation navigation_with_slam.launch.py
```

With this setup, navigation goals can be sent into the partially known map. The robot will plan paths within explored areas and continue mapping as it moves.
