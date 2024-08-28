# Bicycle Model Possible Path Visualization
This documentation is for a simple bicycle model simulation.

## Table of Contents
- [Packages](#packages)
- [Parameter Tuning](#parameter-tuning)
- [Topic/Data Types](#topic-and-data-types)

---

### Packages
- #### **bicycle_bringup**
This package contains necessary configuration files for rviz which is `simu.rviz` file.
- #### **bicycle_description**
This package contains the urdf and config files for the bicycle model. `robot.urdf.xacro` contains the robot definition, `ros2_control.xacro` contains the controller definition for the robot. Inside `config` folder you can find the `bicycle_controller.yaml` file which contains the controller configuration for the robot.
- #### **bicycle_gazebo**
This package contains the main launch file for launching the model in gazebo, rviz, all necessary nodes and controllers. This main launch file is `bicycle_gazebo.launch.py`. This package also contains the `worlds` folder which has a empty world file in it.
- #### **bicycle_model**
This package contains the old nodes for the bicycle model. Only executable important is the `path_publisher.py` which publishes the path for the bicycle model to follow.
- #### **bicycle_sim**
Main simulation nodes are inside this package.
- `sim_controller.py`
This node is responsible for implementing the control algorithm for the bicycle model in simulation environment. 
- `transformer.py`
This node creates a world frame which has a dynamic transformation between itself and the robot base link. This is necessary for the bicycle model to move visually.

- `vel_publisher.py`
This node is responsible for publishing the velocity commands to the bicycle model and also visualizing the routes node.

- `visualize_routes.py`
This node is responsible for visualizing the routes for the bicycle model might follow for the given steering angle and velocity configurations.

### Parameter Tuning
Since this bicycle model is not a real model, i.e we do not consider any dynamics of the bicycle model(to achieve this bicycle model is created such that it's weight is as small as possible allowing us to "ignore" the dynamics of the system.) 

In this simulation increasing the velocity higher than 2.5 m/s might cause the bicycle model to flip over. This is because the bicycle model is not stable at higher velocities.

### Topic and Data Types
Important executables are listed below:
- **`sim_controller.py`**
  - Subscribed Topics:
    - **`/odom`** Has **[nav_msgs/Odometry]** data type
    - **`/path`** Has **[nav_msgs/Path]** data type
    - **`/vel_topic`** Has **[std_msgs/Float64]** data type
  - Published Topics:
    - **`/position_controller/commands`** Has **[std_msgs/Float64MultiArray]** data type
    - **`/velocity_controller/commands`** Has **[std_msgs/Float64MultiArray]** data type

- **`transformer.py`**
    - Subscribed Topics:
      - **`/odom`** Has **[nav_msgs/Odometry]** data type

- **`vel_publisher.py`**
    - Published Topics:
      - **`/vel_topic`** Has **[std_msgs/Float64]** data type

- **`visualize_routes.py`**
    - Subscribed Topics:
      - **`/odom`** Has **[nav_msgs/Odometry]** data type
      - **`/vel_topic`** Has **[std_msgs/Float64]** data type
    - Published Topics:
      - **`/marker_array`** Has **[visualization_msgs/MarkerArray]** data type

- **`path_publisher.py`**
    - Published Topics:
      - **`/path`** Has **[nav_msgs/Path]** data type

Generally speaking each topic is doing the following:
- **`/odom`** is to gather odometry data of the bicycle model.
- **`/path`** is to publish the path for the bicycle model to follow.
- **`/vel_topic`** is to publish the velocity commands to the bicycle model and to visualize the possible routes.
- **`/position_controller/commands`** is to publish the position commands to the bicycle model i.e. publishing steering commands.
- **`/velocity_controller/commands`** is to publish the velocity commands to the bicycle model for the front and rear wheels.
- **`/marker_array`** is to publish the visualization of the possible routes for the bicycle model can follow.

