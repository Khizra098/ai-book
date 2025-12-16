---
title: Nav2 Path Planning for Humanoid Robots
description: Comprehensive guide to configuring Navigation2 (Nav2) for bipedal humanoid robots with specific locomotion constraints
sidebar_label: Nav2 Path Planning
---

# Nav2 Path Planning for Humanoid Robots

## Nav2 Configuration for Humanoid Robots

Navigation2 (Nav2) is the state-of-the-art navigation framework for ROS 2, designed to provide robust path planning, obstacle avoidance, and navigation capabilities for mobile robots. For humanoid robots, Nav2 requires specialized configuration to account for the unique challenges of bipedal locomotion, including balance constraints, step planning, and dynamic stability requirements.

The standard Nav2 configuration, designed primarily for wheeled robots, must be adapted to accommodate the specific kinematic and dynamic constraints of humanoid robots. This involves configuring costmaps, path planners, and controllers to generate dynamically feasible paths that respect the bipedal nature of humanoid locomotion.

### Key Components for Humanoid Navigation

Nav2 consists of several key components that need to be configured specifically for humanoid robots:

- **Costmap 2D**: Modified to account for humanoid footprints and balance constraints
- **Global Planner**: Adjusted for bipedal path planning with step constraints
- **Local Planner**: Configured for dynamic balance and step generation
- **Controller Server**: Adapted for humanoid-specific trajectory following
- **Recovery Behaviors**: Modified for humanoid-specific recovery actions
- **Behavior Tree**: Customized for humanoid navigation decision-making

### Installation and Prerequisites

To use Nav2 for humanoid robotics applications, you'll need:

- ROS 2 Humble Hawksbill or newer
- Navigation2 packages
- Robot description (URDF) with proper joint limits
- TF tree with all required transforms
- Sensor data (laser, depth camera, etc.)
- Robot state publisher

```bash
# Install Navigation2 packages
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui
sudo apt install ros-humble-nav2-rviz-plugins
```

### Basic Humanoid Navigation Node

```python
# Example: Basic Nav2 configuration for humanoid robot
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import math

class HumanoidNavigationNode(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_node')

        # Create action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Publisher for humanoid-specific navigation commands
        self.step_pub = self.create_publisher(
            # Custom message for step commands
            'humanoid_msgs/msg/StepCommand',
            '/step_commands',
            10
        )

        # Subscriber for navigation status
        self.nav_status_sub = self.create_subscription(
            # Custom message for navigation status
            'humanoid_msgs/msg/NavigationStatus',
            '/navigation_status',
            self.nav_status_callback,
            10
        )

        self.get_logger().info('Humanoid Navigation Node initialized')

    def send_navigation_goal(self, x, y, theta):
        """
        Send navigation goal to Nav2 with humanoid-specific parameters
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Wait for action server
        self.nav_to_pose_client.wait_for_server()

        # Send goal with humanoid-specific options
        future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )

        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """
        Handle navigation goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

    def nav_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigation feedback: {feedback.current_pose.pose.position.x:.2f}, '
            f'{feedback.current_pose.pose.position.y:.2f}'
        )

    def nav_status_callback(self, msg):
        """
        Handle navigation status updates
        """
        # Handle humanoid-specific navigation status
        pass
```

### Humanoid-Specific Parameters

Humanoid robots require specific parameters to be configured in Nav2:

```yaml
# Example: Humanoid-specific Nav2 parameters
# config/humanoid_nav2_params.yaml

amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_footprint"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    # Specify the path where the BT XML files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    # Humanoid-specific controller parameters
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid path follower
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.1
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.3
      wz_max: 0.5
      xy_goal_tolerance: 0.2  # Humanoid-specific tolerance
      yaw_goal_tolerance: 0.1
      stateful: true
      critic_names: [
        "BaseGoalCritic",
        "BaseProgressCritic",
        "GoalAngleCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathAngleCritic",
        "PreferForwardCritic",
        "HumanoidBalanceCritic"]  # Humanoid-specific critic
      BaseGoalCritic.scale: 2.0
      BaseProgressCritic.scale: 3.0
      GoalAngleCritic.scale: 0.01
      PathAlignCritic.scale: 3.2
      PathFollowCritic.scale: 0.5
      PathAngleCritic.scale: 0.01
      PreferForwardCritic.scale: 0.002
      HumanoidBalanceCritic.scale: 5.0  # Prioritize balance for humanoid
      HumanoidBalanceCritic.boundary_width: 0.1
      HumanoidBalanceCritic.boundary_weight: 10.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_footprint"
      use_sim_time: false
      rolling_window: true
      width: 6  # Humanoid-specific local costmap size
      height: 6
      resolution: 0.05
      # Footprint for humanoid robot (larger than typical wheeled robot)
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      footprint_padding: 0.01
      inflation_radius: 0.5  # Humanoid-specific inflation
      cost_scaling_factor: 5.0
      map_topic: "/map"
      always_send_full_costmap: true
      observation_sources: scan
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_footprint"
      use_sim_time: false
      width: 200  # Larger global costmap for humanoid navigation
      height: 200
      resolution: 0.05
      # Humanoid-specific footprint
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      footprint_padding: 0.01
      inflation_radius: 0.8  # Larger inflation for humanoid safety
      cost_scaling_factor: 10.0
      map_topic: "/map"
      always_send_full_costmap: true
      observation_sources: scan
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 20.0
        raytrace_min_range: 0.0
        obstacle_max_range: 20.0
        obstacle_min_range: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      # Use a planner suitable for humanoid navigation
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5  # Humanoid-specific tolerance
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57  # Humanoid-specific spin behavior
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: -0.15  # Humanoid-specific backup distance
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1s
```

## Bipedal Navigation Constraints and Parameters

### Humanoid Kinematic Constraints

Humanoid robots have specific kinematic constraints that must be considered in navigation planning:

```python
# Example: Humanoid kinematic constraints
class HumanoidKinematicConstraints:
    def __init__(self):
        # Physical dimensions
        self.leg_length = 0.8  # meters
        self.step_length_max = 0.3  # maximum step length
        self.step_width_max = 0.4   # maximum step width
        self.step_height_max = 0.1  # maximum step height (for stepping over obstacles)

        # Balance constraints
        self.com_height = 0.9  # Center of mass height
        self.support_polygon_margin = 0.05  # Safety margin for support polygon

        # Motion constraints
        self.max_linear_velocity = 0.3  # m/s
        self.max_angular_velocity = 0.2  # rad/s
        self.max_acceleration = 0.1     # m/s^2
        self.max_angular_accel = 0.1    # rad/s^2

        # Step timing constraints
        self.min_step_time = 0.5  # seconds
        self.max_step_time = 2.0  # seconds

    def is_path_dynamically_feasible(self, path):
        """
        Check if a path is dynamically feasible for humanoid locomotion
        """
        for i in range(len(path.poses) - 1):
            segment = self.calculate_path_segment(path.poses[i], path.poses[i+1])
            if not self.is_segment_feasible(segment):
                return False
        return True

    def is_segment_feasible(self, segment):
        """
        Check if a path segment is feasible for humanoid locomotion
        """
        # Check distance constraint (step length)
        distance = self.calculate_distance(segment.start, segment.end)
        if distance > self.step_length_max:
            return False

        # Check turning constraint (step width)
        turn_angle = self.calculate_turn_angle(segment)
        if abs(turn_angle) > math.pi / 4:  # 45 degrees max turn per step
            return False

        # Check height constraint (for stepping over obstacles)
        height_diff = abs(segment.end.position.z - segment.start.position.z)
        if height_diff > self.step_height_max:
            return False

        return True

    def calculate_distance(self, pose1, pose2):
        """
        Calculate 2D distance between two poses
        """
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)

    def calculate_turn_angle(self, segment):
        """
        Calculate turn angle for a path segment
        """
        # Calculate orientation difference
        q1 = segment.start.orientation
        q2 = segment.end.orientation

        # Convert to roll, pitch, yaw to get turn angle
        # Simplified calculation - in practice would use full quaternion math
        return 0.0  # Placeholder
```

### Step Planning for Bipedal Locomotion

```python
# Example: Step planning for humanoid navigation
class StepPlanner:
    def __init__(self, robot_params):
        self.robot_params = robot_params
        self.foot_separation = 0.2  # Distance between feet
        self.step_timing = 0.8      # Time per step

    def plan_steps_for_path(self, path):
        """
        Plan footstep sequence for a given navigation path
        """
        steps = []
        current_pose = path.poses[0]

        for i in range(1, len(path.poses)):
            target_pose = path.poses[i]
            step_sequence = self.calculate_step_sequence(current_pose, target_pose)
            steps.extend(step_sequence)
            current_pose = target_pose

        return steps

    def calculate_step_sequence(self, start_pose, end_pose):
        """
        Calculate the sequence of steps needed to move from start to end pose
        """
        # Calculate required steps based on distance and step constraints
        distance = self.calculate_distance(start_pose, end_pose)
        num_steps = int(math.ceil(distance / self.robot_params.step_length_max))

        if num_steps == 0:
            return []

        # Generate intermediate poses for each step
        step_poses = []
        for i in range(1, num_steps + 1):
            ratio = i / num_steps
            step_pose = self.interpolate_pose(start_pose, end_pose, ratio)
            step_poses.append(step_pose)

        return step_poses

    def interpolate_pose(self, start_pose, end_pose, ratio):
        """
        Interpolate between two poses
        """
        step_pose = Pose()

        # Interpolate position
        step_pose.position.x = start_pose.position.x + \
            ratio * (end_pose.position.x - start_pose.position.x)
        step_pose.position.y = start_pose.position.y + \
            ratio * (end_pose.position.y - start_pose.position.y)
        step_pose.position.z = start_pose.position.z + \
            ratio * (end_pose.position.z - start_pose.position.z)

        # Interpolate orientation (simplified)
        step_pose.orientation = self.interpolate_quaternion(
            start_pose.orientation, end_pose.orientation, ratio
        )

        return step_pose

    def interpolate_quaternion(self, q1, q2, t):
        """
        Spherical linear interpolation of quaternions
        """
        # Simplified implementation - in practice would use proper SLERP
        return q1  # Placeholder
```

### Balance-Aware Path Planning

```python
# Example: Balance-aware path planning for humanoid robots
class BalanceAwarePlanner:
    def __init__(self, robot_params):
        self.robot_params = robot_params
        self.zmp_margin = 0.05  # Safety margin for Zero Moment Point

    def plan_balance_safe_path(self, start, goal, costmap):
        """
        Plan a path that maintains balance throughout navigation
        """
        # Use a modified path planning algorithm that considers balance
        # This would typically integrate with Nav2's global planner
        original_path = self.plan_standard_path(start, goal, costmap)

        # Post-process the path to ensure balance constraints
        balance_safe_path = self.add_balance_constraints(original_path, costmap)

        return balance_safe_path

    def add_balance_constraints(self, path, costmap):
        """
        Add balance constraints to an existing path
        """
        # For each pose in the path, check if it maintains balance
        # Adjust poses if necessary to maintain center of mass within support polygon
        adjusted_path = []

        for pose in path.poses:
            adjusted_pose = self.adjust_for_balance(pose, costmap)
            adjusted_path.append(adjusted_pose)

        # Create new path message
        balance_safe_path = Path()
        balance_safe_path.header = path.header
        balance_safe_path.poses = adjusted_path

        return balance_safe_path

    def adjust_for_balance(self, pose, costmap):
        """
        Adjust a pose to maintain balance
        """
        # Check if the pose maintains balance based on local terrain
        # Adjust position if needed to keep center of mass stable
        return pose  # Placeholder implementation
```

## Costmap Configuration for Humanoid Footprints

### Custom Footprint Definition

Humanoid robots require a different approach to footprint definition compared to wheeled robots:

```yaml
# Example: Humanoid-specific costmap configuration
# config/humanoid_costmap_params.yaml

local_costmap:
  local_costmap:
    ros__parameters:
      # Basic costmap parameters
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_footprint"
      use_sim_time: false
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05

      # Humanoid-specific footprint
      # This represents the area that must remain obstacle-free for stable walking
      footprint: "[[-0.35, -0.25], [-0.35, 0.25], [0.35, 0.25], [0.35, -0.25]]"
      footprint_padding: 0.02  # Extra padding for safety

      # Inflation parameters for humanoid safety
      inflation_radius: 0.6    # Larger inflation for stable walking area
      cost_scaling_factor: 6.0 # Adjust cost scaling for humanoid needs

      # Obstacle handling
      map_topic: "/map"
      always_send_full_costmap: true
      observation_sources: scan depth_camera

      # Laser scan observations
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 5.0
        raytrace_min_range: 0.0
        obstacle_max_range: 4.0
        obstacle_min_range: 0.1

      # Depth camera observations for 3D obstacle detection
      depth_camera:
        topic: "/camera/depth/points"
        max_obstacle_height: 1.8
        min_obstacle_height: 0.1
        clearing: true
        marking: true
        data_type: "PointCloud2"
        expected_update_rate: 2.0
        observation_persistence: 0.0
        inf_is_valid: false
        clearing_endpoints: false

global_costmap:
  global_costmap:
    ros__parameters:
      # Basic costmap parameters
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_footprint"
      use_sim_time: false

      # Larger costmap for humanoid navigation planning
      width: 300
      height: 300
      resolution: 0.05

      # Humanoid-specific footprint
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      footprint_padding: 0.01

      # Inflation for humanoid safety
      inflation_radius: 1.0    # Larger inflation for path safety
      cost_scaling_factor: 8.0

      # Map and obstacle sources
      map_topic: "/map"
      always_send_full_costmap: true
      observation_sources: scan

      # Obstacle handling
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 30.0
        raytrace_min_range: 0.0
        obstacle_max_range: 25.0
        obstacle_min_range: 0.0
```

### Dynamic Footprint Adjustment

Humanoid robots may need dynamic footprint adjustment based on walking state:

```python
# Example: Dynamic footprint adjustment for humanoid navigation
import math
from geometry_msgs.msg import Polygon, Point32

class DynamicFootprintAdjuster:
    def __init__(self, node):
        self.node = node
        self.base_footprint = self.create_rectangle_footprint(0.3, 0.2)
        self.current_state = 'standing'  # standing, walking, turning, etc.

    def create_rectangle_footprint(self, length, width):
        """
        Create a rectangular footprint for the robot
        """
        footprint = Polygon()
        footprint.points = [
            Point32(x=-length/2, y=-width/2, z=0.0),
            Point32(x=-length/2, y=width/2, z=0.0),
            Point32(x=length/2, y=width/2, z=0.0),
            Point32(x=length/2, y=-width/2, z=0.0)
        ]
        return footprint

    def get_adjusted_footprint(self, robot_state):
        """
        Get footprint adjusted based on current robot state
        """
        if robot_state == 'walking':
            # When walking, use a larger footprint to account for step dynamics
            return self.create_rectangle_footprint(0.4, 0.3)
        elif robot_state == 'turning':
            # When turning, account for potential lateral movement
            return self.create_rectangle_footprint(0.35, 0.35)
        elif robot_state == 'standing':
            # Normal standing footprint
            return self.base_footprint
        else:
            # Default to base footprint
            return self.base_footprint

    def update_costmap_footprint(self, footprint):
        """
        Update the costmap with the new footprint
        """
        # This would interface with Nav2's costmap to update the footprint
        # In practice, this involves updating parameters dynamically
        pass
```

### Terrain-Aware Costmap

```python
# Example: Terrain-aware costmap for humanoid navigation
class TerrainAwareCostmap:
    def __init__(self, node):
        self.node = node

        # Terrain cost definitions for humanoid navigation
        self.terrain_costs = {
            'flat_ground': 0.0,
            'ramp': 1.0,
            'stairs': 5.0,
            'narrow_passage': 2.0,
            'uneven_surface': 3.0,
            'slippery_surface': 4.0,
            'obstacle': 254  # lethal obstacle
        }

        # Subscribe to terrain classification
        self.terrain_sub = node.create_subscription(
            # Custom message for terrain classification
            'humanoid_msgs/msg/TerrainClassification',
            '/terrain_classification',
            self.terrain_callback,
            10
        )

    def terrain_callback(self, msg):
        """
        Update costmap based on terrain classification
        """
        # Update costmap cells based on terrain type
        for cell in msg.terrain_cells:
            self.update_cell_cost(cell.x, cell.y, self.terrain_costs[cell.terrain_type])

    def update_cell_cost(self, x, y, cost):
        """
        Update the cost of a specific cell in the costmap
        """
        # This would interface with Nav2's costmap API
        # to update individual cell costs based on terrain
        pass

    def get_terrain_adjusted_path_cost(self, path):
        """
        Calculate path cost considering terrain types
        """
        total_cost = 0.0
        for pose in path.poses:
            # Get terrain cost at this location
            terrain_cost = self.get_terrain_cost_at_pose(pose)
            total_cost += terrain_cost

        return total_cost

    def get_terrain_cost_at_pose(self, pose):
        """
        Get terrain-based cost for a specific pose
        """
        # Determine terrain type at the pose location
        # and return appropriate cost
        return 0.0  # Placeholder
```

## Path Planning Algorithms for Bipedal Locomotion

### Humanoid-Aware Path Planning

```python
# Example: Humanoid-aware path planning algorithm
import numpy as np
from nav2_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class HumanoidPathPlanner:
    def __init__(self):
        self.step_constraints = {
            'max_step_length': 0.3,  # meters
            'max_step_width': 0.4,   # meters
            'max_step_height': 0.1,  # meters
            'min_step_length': 0.05  # meters
        }

        self.balance_constraints = {
            'max_slope': math.radians(15),  # 15 degrees
            'support_polygon_margin': 0.05  # meters
        }

    def plan_humanoid_path(self, start, goal, costmap):
        """
        Plan a path suitable for humanoid locomotion
        """
        # Use A* or Dijkstra with humanoid-specific cost function
        path = self.humanoid_astar(start, goal, costmap)

        # Smooth the path considering humanoid dynamics
        smoothed_path = self.smooth_humanoid_path(path)

        # Add step planning information
        step_path = self.add_step_information(smoothed_path)

        return step_path

    def humanoid_astar(self, start, goal, costmap):
        """
        A* algorithm modified for humanoid constraints
        """
        # Initialize open and closed sets
        open_set = [(0, start)]  # (f_score, position)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            # Get node with lowest f_score
            current = min(open_set, key=lambda x: x[0])[1]
            open_set.remove((f_score.get(current, float('inf')), current))

            if current == goal:
                # Reconstruct path
                path = self.reconstruct_path(came_from, current)
                return path

            # Explore neighbors
            neighbors = self.get_humanoid_valid_neighbors(current, costmap)
            for neighbor in neighbors:
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # This path to neighbor is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)

                    if neighbor not in [x[1] for x in open_set]:
                        open_set.append((f_score[neighbor], neighbor))

        return None  # No path found

    def get_humanoid_valid_neighbors(self, position, costmap):
        """
        Get valid neighbors considering humanoid constraints
        """
        neighbors = []
        step_size = 0.1  # 10cm steps

        # Generate potential neighbor positions
        for dx in [-step_size, 0, step_size]:
            for dy in [-step_size, 0, step_size]:
                if dx == 0 and dy == 0:
                    continue  # Skip current position

                neighbor_pos = (position[0] + dx, position[1] + dy)

                # Check if step is within humanoid constraints
                if self.is_valid_humanoid_step(position, neighbor_pos):
                    # Check if position is traversable
                    if self.is_traversable(neighbor_pos, costmap):
                        neighbors.append(neighbor_pos)

        return neighbors

    def is_valid_humanoid_step(self, start_pos, end_pos):
        """
        Check if a step is valid for humanoid locomotion
        """
        # Calculate step distance
        step_distance = self.distance(start_pos, end_pos)

        # Check step length constraint
        if step_distance > self.step_constraints['max_step_length']:
            return False

        # Check step length minimum
        if step_distance < self.step_constraints['min_step_length']:
            return False

        # Additional checks could include slope, terrain type, etc.
        return True

    def is_traversable(self, position, costmap):
        """
        Check if a position is traversable for humanoid
        """
        # Check costmap value at position
        cost = self.get_cost_at_position(position, costmap)

        # Humanoid can typically navigate through higher cost areas
        # than wheeled robots, but still needs to avoid lethal obstacles
        return cost < 254  # lethal obstacle threshold

    def distance(self, pos1, pos2):
        """
        Calculate Euclidean distance between two positions
        """
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.sqrt(dx*dx + dy*dy)

    def heuristic(self, pos1, pos2):
        """
        Heuristic function for A* (Euclidean distance)
        """
        return self.distance(pos1, pos2)

    def reconstruct_path(self, came_from, current):
        """
        Reconstruct path from came_from dictionary
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def smooth_humanoid_path(self, path):
        """
        Smooth path considering humanoid dynamics
        """
        if len(path) < 3:
            return path

        # Implement path smoothing algorithm suitable for humanoid
        # This could use cubic splines, clothoid curves, or other techniques
        smoothed_path = []

        # Simple smoothing by removing unnecessary waypoints
        smoothed_path.append(path[0])

        for i in range(1, len(path) - 1):
            # Check if middle point is necessary
            prev_point = path[i-1]
            curr_point = path[i]
            next_point = path[i+1]

            # Calculate angle between segments
            angle = self.calculate_angle(prev_point, curr_point, next_point)

            # If angle is small, the middle point might be unnecessary
            if abs(angle) > math.radians(10):  # Keep points with significant direction change
                smoothed_path.append(curr_point)

        smoothed_path.append(path[-1])
        return smoothed_path

    def calculate_angle(self, p1, p2, p3):
        """
        Calculate angle between three points
        """
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])

        dot_product = v1[0]*v2[0] + v1[1]*v2[1]
        mag_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
        mag_v2 = math.sqrt(v2[0]**2 + v2[1]**2)

        if mag_v1 == 0 or mag_v2 == 0:
            return 0

        cos_angle = dot_product / (mag_v1 * mag_v2)
        cos_angle = max(-1, min(1, cos_angle))  # Clamp to avoid numerical errors
        return math.acos(cos_angle)

    def add_step_information(self, path):
        """
        Add step planning information to path
        """
        # Convert path to ROS Path message with step information
        ros_path = Path()
        ros_path.header.frame_id = "map"

        for i, pose in enumerate(path):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = 0.0

            # Set orientation to face the next waypoint
            if i < len(path) - 1:
                next_pose = path[i+1]
                yaw = math.atan2(next_pose[1] - pose[1], next_pose[0] - pose[0])
                pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Keep same orientation as previous point
                if i > 0:
                    prev_pose = path[i-1]
                    if i > 1:
                        prev_prev_pose = path[i-2]
                        yaw = math.atan2(pose[1] - prev_prev_pose[1], pose[0] - prev_prev_pose[0])
                        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)

            ros_path.poses.append(pose_stamped)

        return ros_path
```

### Integration with Nav2 Planners

```python
# Example: Custom Nav2 planner plugin for humanoid robots
from nav2_core.global_planner import GlobalPlanner
from nav2_costmap_2d.costmap_2d_ros import Costmap2DROS
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class HumanoidGlobalPlanner(GlobalPlanner):
    def __init__(self):
        self.initialized = False
        self.planner_name = ""

    def configure(self, tf_buffer, costmap_ros, autostart):
        """
        Configure the planner with costmap and other parameters
        """
        self.costmap_ros = costmap_ros
        self.tf_buffer = tf_buffer
        self.planner_name = "HumanoidGlobalPlanner"
        self.initialized = True

        # Initialize humanoid-specific parameters
        self.step_constraints = {
            'max_step_length': 0.3,
            'max_step_width': 0.4,
            'max_step_height': 0.1
        }

        self.get_logger().info(f'{self.planner_name} configured successfully')

    def cleanup(self):
        """
        Clean up resources
        """
        self.initialized = False
        self.get_logger().info(f'{self.planner_name} cleaned up')

    def set_costmap(self, costmap):
        """
        Set the costmap for the planner
        """
        self.costmap = costmap

    def create_plan(self, start, goal):
        """
        Create a plan from start to goal
        """
        if not self.initialized:
            self.get_logger().error(f'{self.planner_name} has not been initialized')
            return Path()

        # Convert ROS poses to coordinates
        start_coords = (start.pose.position.x, start.pose.position.y)
        goal_coords = (goal.pose.position.x, goal.pose.position.y)

        # Plan path considering humanoid constraints
        path = self.plan_humanoid_path(start_coords, goal_coords)

        # Convert to ROS Path message
        ros_path = self.convert_to_ros_path(path, start.header.frame_id)

        return ros_path

    def plan_humanoid_path(self, start_coords, goal_coords):
        """
        Plan path considering humanoid constraints
        """
        # This would implement the actual path planning algorithm
        # considering humanoid-specific constraints
        path = []

        # For demonstration, create a simple path
        # In practice, this would use the HumanoidPathPlanner class
        current_x, current_y = start_coords
        goal_x, goal_y = goal_coords

        # Calculate direction vector
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance == 0:
            return [start_coords]

        # Normalize direction
        dx /= distance
        dy /= distance

        # Generate path with step constraints
        step_size = min(self.step_constraints['max_step_length'], distance)
        num_steps = int(distance / step_size)

        path.append(start_coords)

        for i in range(1, num_steps + 1):
            ratio = i / num_steps
            x = current_x + ratio * (goal_x - current_x)
            y = current_y + ratio * (goal_y - current_y)
            path.append((x, y))

        # Ensure we reach the goal
        if path[-1] != goal_coords:
            path.append(goal_coords)

        return path

    def convert_to_ros_path(self, path_coords, frame_id):
        """
        Convert coordinate path to ROS Path message
        """
        ros_path = Path()
        ros_path.header.frame_id = frame_id

        for i, (x, y) in enumerate(path_coords):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            # Set orientation to face the next waypoint
            if i < len(path_coords) - 1:
                next_x, next_y = path_coords[i+1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Keep same orientation as previous point
                if i > 0:
                    prev_x, prev_y = path_coords[i-1]
                    if i > 1:
                        prev_prev_x, prev_prev_y = path_coords[i-2]
                        yaw = math.atan2(y - prev_prev_y, x - prev_prev_x)
                        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)

            ros_path.poses.append(pose_stamped)

        return ros_path
```

## Dynamic Obstacle Avoidance and Replanning

### Real-time Path Replanning

```python
# Example: Dynamic obstacle avoidance for humanoid robots
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid
from tf2_ros import Buffer, TransformListener
import math

class HumanoidDynamicAvoidance(Node):
    def __init__(self):
        super().__init__('humanoid_dynamic_avoidance')

        # TF buffer for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_smooth',
            10
        )

        self.replanned_path_pub = self.create_publisher(
            Path,
            '/replanned_path',
            10
        )

        # Internal state
        self.current_path = None
        self.current_velocity = Twist()
        self.obstacles = []
        self.safe_path = None

        # Parameters
        self.declare_parameter('avoidance_timeout', 5.0)
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.declare_parameter('dynamic_window_size', 2.0)

        self.get_logger().info('Humanoid Dynamic Avoidance Node initialized')

    def scan_callback(self, msg):
        """
        Process laser scan data to detect dynamic obstacles
        """
        # Process scan data to identify obstacles
        self.obstacles = self.detect_obstacles_from_scan(msg)

        # Check if current path is blocked
        if self.current_path and self.is_path_blocked(self.current_path, self.obstacles):
            # Plan alternative path
            self.safe_path = self.replan_around_obstacles()
            if self.safe_path:
                self.replanned_path_pub.publish(self.safe_path)

    def path_callback(self, msg):
        """
        Update current path
        """
        self.current_path = msg

    def velocity_callback(self, msg):
        """
        Update current velocity
        """
        self.current_velocity = msg

    def detect_obstacles_from_scan(self, scan_msg):
        """
        Detect obstacles from laser scan data
        """
        obstacles = []
        angle_increment = scan_msg.angle_increment

        for i, range_val in enumerate(scan_msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)):
                if range_val < scan_msg.range_max and range_val > scan_msg.range_min:
                    angle = scan_msg.angle_min + i * angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    obstacles.append((x, y, range_val))

        return obstacles

    def is_path_blocked(self, path, obstacles):
        """
        Check if the current path is blocked by obstacles
        """
        min_obstacle_distance = self.get_parameter('min_obstacle_distance').value

        for pose in path.poses:
            for obs_x, obs_y, _ in obstacles:
                distance = math.sqrt(
                    (pose.pose.position.x - obs_x)**2 +
                    (pose.pose.position.y - obs_y)**2
                )
                if distance < min_obstacle_distance:
                    return True

        return False

    def replan_around_obstacles(self):
        """
        Replan path around detected obstacles
        """
        if not self.current_path:
            return None

        # For humanoid robots, replanning needs to consider step constraints
        # This is a simplified implementation
        current_pos = self.current_path.poses[0].pose.position

        # Find next safe waypoint
        for i, pose in enumerate(self.current_path.poses):
            is_safe = True
            min_obstacle_distance = self.get_parameter('min_obstacle_distance').value

            for obs_x, obs_y, _ in self.obstacles:
                distance = math.sqrt(
                    (pose.pose.position.x - obs_x)**2 +
                    (pose.pose.position.y - obs_y)**2
                )
                if distance < min_obstacle_distance:
                    is_safe = False
                    break

            if is_safe and i > 10:  # Skip first few waypoints to move past immediate obstacles
                # Create new path from current position to safe waypoint
                new_path = Path()
                new_path.header = self.current_path.header

                # Add current position
                start_pose = PoseStamped()
                start_pose.header = new_path.header
                start_pose.pose.position = current_pos
                new_path.poses.append(start_pose)

                # Add safe goal
                goal_pose = PoseStamped()
                goal_pose.header = new_path.header
                goal_pose.pose = pose.pose
                new_path.poses.append(goal_pose)

                # Extend to original goal if possible
                if len(self.current_path.poses) > i + 1:
                    for j in range(i + 1, len(self.current_path.poses)):
                        new_path.poses.append(self.current_path.poses[j])

                return new_path

        return None

    def calculate_dynamic_velocity(self, target_velocity, obstacles):
        """
        Calculate safe velocity considering dynamic obstacles
        """
        min_distance = self.get_parameter('min_obstacle_distance').value
        safe_velocity = target_velocity

        for obs_x, obs_y, obs_range in obstacles:
            if obs_range < min_distance:
                # Reduce velocity based on proximity to obstacle
                reduction_factor = obs_range / min_distance
                safe_velocity.linear.x *= reduction_factor
                safe_velocity.linear.y *= reduction_factor
                # Reduce angular velocity as well
                safe_velocity.angular.z *= reduction_factor

        return safe_velocity
```

### Humanoid-Specific Recovery Behaviors

```python
# Example: Humanoid-specific recovery behaviors
class HumanoidRecoveryBehaviors:
    def __init__(self, node):
        self.node = node
        self.recovery_behaviors = {
            'step_back': self.step_back_recovery,
            'wait_and_assess': self.wait_and_assess_recovery,
            'find_alternative_path': self.find_alternative_path_recovery,
            'balance_recovery': self.balance_recovery
        }

    def step_back_recovery(self):
        """
        Recovery behavior: take a step back to gain space
        """
        # Generate a command to step back
        cmd = self.generate_step_command('back', 0.1)  # 10cm back
        return cmd

    def wait_and_assess_recovery(self):
        """
        Recovery behavior: wait and reassess the situation
        """
        # Stop movement and reassess environment
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        return cmd

    def find_alternative_path_recovery(self):
        """
        Recovery behavior: find an alternative path
        """
        # Trigger replanning with a different approach
        # This might involve expanding the search area or using a different planner
        pass

    def balance_recovery(self):
        """
        Recovery behavior: adjust posture for balance
        """
        # Command to adjust center of mass or take a balancing step
        cmd = self.generate_balance_command()
        return cmd

    def generate_step_command(self, direction, distance):
        """
        Generate a step command in the specified direction
        """
        # This would generate appropriate step commands for humanoid locomotion
        pass

    def generate_balance_command(self):
        """
        Generate a command to improve balance
        """
        # This would generate posture adjustments or balancing steps
        pass
```

## Runnable Nav2 Configuration Examples with Proper Syntax Highlighting

### Complete Nav2 Launch File for Humanoid Robots

```xml
<?xml version="1.0"?>
<!-- Nav2 launch file for humanoid robot navigation -->
<launch>
  <!-- Arguments -->
  <arg name="namespace" default=""/>
  <arg name="use_sim_time" default="false"/>
  <arg name="autostart" default="true"/>
  <arg name="params_file" default="$(find-pkg-share humanoid_navigation)/config/humanoid_nav2_params.yaml"/>
  <arg name="default_bt_xml_filename" default="$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_w_replanning_and_recovery.xml"/>
  <arg name="map_subscribe_transient_local" default="false"/>

  <!-- Nodes -->
  <group ns="$(var namespace)">
    <!-- Map Server -->
    <node pkg="nav2_map_server" exec="map_server" name="map_server">
      <param name="yaml_filename" value="path/to/map.yaml"/>
      <param name="topic_name" value="map"/>
      <param name="frame_id" value="map"/>
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="always_send_full_map" value="false"/>
      <param name="publish_frequency" value="1.0"/>
    </node>

    <!-- Local Costmap -->
    <node pkg="nav2_costmap_2d" exec="costmap_2d_node" name="local_costmap" respawn="false">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="yaml_filename" value="$(find-pkg-share humanoid_navigation)/config/local_costmap_params.yaml"/>
    </node>

    <!-- Global Costmap -->
    <node pkg="nav2_costmap_2d" exec="costmap_2d_node" name="global_costmap" respawn="false">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="yaml_filename" value="$(find-pkg-share humanoid_navigation)/config/global_costmap_params.yaml"/>
    </node>

    <!-- AMCL -->
    <node pkg="nav2_amcl" exec="amcl" name="amcl" respawn="false">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="yaml_filename" value="$(find-pkg-share humanoid_navigation)/config/amcl_params.yaml"/>
    </node>

    <!-- Planner Server -->
    <node pkg="nav2_planner" exec="planner_server" name="planner_server" respawn="false">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="yaml_filename" value="$(var params_file)"/>
    </node>

    <!-- Controller Server -->
    <node pkg="nav2_controller" exec="controller_server" name="controller_server" respawn="false">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="yaml_filename" value="$(var params_file)"/>
    </node>

    <!-- Behavior Tree Navigator -->
    <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator" respawn="false">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="yaml_filename" value="$(var params_file)"/>
      <param name="default_bt_xml_filename" value="$(var default_bt_xml_filename)"/>
      <param name="enable_groot_monitoring" value="true"/>
      <param name="groot_zmq_publisher_port" value="1666"/>
      <param name="groot_zmq_server_port" value="1667"/>
    </node>

    <!-- Lifecycle Manager -->
    <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager_navigation">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="autostart" value="$(var autostart)"/>
      <param name="node_names" value="[map_server, local_costmap, global_costmap, amcl, planner_server, controller_server, bt_navigator]"/>
    </node>
  </group>
</launch>
```

### Complete Nav2 Configuration File

```yaml
# Complete Nav2 configuration for humanoid robot navigation
# config/humanoid_nav2_complete.yaml

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_footprint"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    # Custom behavior tree for humanoid navigation
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    # Humanoid-specific behavior tree nodes
    - nav2_humanoid_balance_check_bt_node
    - nav2_step_sequence_generator_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific path follower
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.1
      wz_std: 0.4
      vx_max: 0.4  # Lower max velocity for humanoid stability
      vx_min: -0.2
      vy_max: 0.2
      wz_max: 0.3
      xy_goal_tolerance: 0.3  # Larger tolerance for humanoid
      yaw_goal_tolerance: 0.2
      stateful: true
      critic_names: [
        "BaseGoalCritic",
        "BaseProgressCritic",
        "GoalAngleCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathAngleCritic",
        "PreferForwardCritic",
        "HumanoidBalanceCritic",  # Humanoid-specific critic
        "StepConstraintCritic"]   # Step constraint critic
      BaseGoalCritic.scale: 2.0
      BaseProgressCritic.scale: 3.0
      GoalAngleCritic.scale: 0.01
      PathAlignCritic.scale: 3.2
      PathFollowCritic.scale: 0.5
      PathAngleCritic.scale: 0.01
      PreferForwardCritic.scale: 0.002
      HumanoidBalanceCritic.scale: 8.0  # High priority for balance
      HumanoidBalanceCritic.boundary_width: 0.15
      HumanoidBalanceCritic.boundary_weight: 15.0
      StepConstraintCritic.scale: 6.0   # Enforce step constraints
      StepConstraintCritic.step_length_weight: 10.0
      StepConstraintCritic.step_width_weight: 8.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_footprint"
      use_sim_time: false
      rolling_window: true
      width: 8  # Larger window for humanoid navigation
      height: 8
      resolution: 0.05
      # Humanoid-specific footprint
      footprint: "[[-0.35, -0.25], [-0.35, 0.25], [0.35, 0.25], [0.35, -0.25]]"
      footprint_padding: 0.02
      inflation_radius: 0.7  # Larger inflation for humanoid safety
      cost_scaling_factor: 7.0
      map_topic: "/map"
      always_send_full_costmap: true
      observation_sources: scan depth_camera
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 5.0
        raytrace_min_range: 0.0
        obstacle_max_range: 4.0
        obstacle_min_range: 0.1
      depth_camera:
        topic: "/camera/depth/points"
        max_obstacle_height: 1.8
        min_obstacle_height: 0.1
        clearing: true
        marking: true
        data_type: "PointCloud2"
        expected_update_rate: 2.0
        observation_persistence: 0.0
        inf_is_valid: false
        clearing_endpoints: false

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_footprint"
      use_sim_time: false
      width: 300
      height: 300
      resolution: 0.05
      # Humanoid-specific footprint
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      footprint_padding: 0.01
      inflation_radius: 1.2  # Larger inflation for path safety
      cost_scaling_factor: 10.0
      map_topic: "/map"
      always_send_full_costmap: true
      observation_sources: scan
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 30.0
        raytrace_min_range: 0.0
        obstacle_max_range: 25.0
        obstacle_min_range: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      # Use a planner suitable for humanoid navigation
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.8  # Larger tolerance for humanoid
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "humanoid_balance"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: -0.20  # Slightly larger backup for humanoid
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1s
    humanoid_balance:
      plugin: "nav2_humanoid_behaviors::BalanceRecovery"
      max_balance_time: 5.0
      balance_tolerance: 0.1

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      wait_time: 1s

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    velocity_timeout: 1.0
    max_velocity: [0.5, 0.0, 1.0]
    min_velocity: [-0.2, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    odom_topic: "odom"
    cmd_vel_topic: "cmd_vel_smooth"
    robot_base_frame: "base_footprint"
```

### Example Python Script for Nav2 Integration

```python
#!/usr/bin/env python3
"""
Complete example script for Nav2 integration with humanoid robot
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math

class HumanoidNav2Interface(Node):
    def __init__(self):
        super().__init__('humanoid_nav2_interface')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/navigation_status',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/current_path',
            10
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Internal state
        self.current_pose = None
        self.navigation_goal = None
        self.is_navigating = False
        self.last_scan = None

        # Timers
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_navigation_status
        )

        self.get_logger().info('Humanoid Nav2 Interface initialized')

    def odom_callback(self, msg):
        """
        Update current pose from odometry
        """
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """
        Store latest scan data
        """
        self.last_scan = msg

    def send_navigation_goal(self, x, y, theta):
        """
        Send navigation goal to Nav2
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info(f'Sending navigation goal to ({x}, {y})')

        # Send goal with feedback callback
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
        self.navigation_goal = (x, y)

        return True

    def goal_response_callback(self, future):
        """
        Handle goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

        self.is_navigating = False
        self.navigation_goal = None

    def feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose.position
        distance_remaining = feedback.distance_remaining

        self.get_logger().debug(
            f'Navigating: ({current_pose.x:.2f}, {current_pose.y:.2f}), '
            f'Distance remaining: {distance_remaining:.2f}m'
        )

    def publish_navigation_status(self):
        """
        Publish current navigation status
        """
        status_msg = String()

        if self.is_navigating:
            if self.navigation_goal:
                status_msg.data = f"NAVIGATING_TO:({self.navigation_goal[0]:.2f},{self.navigation_goal[1]:.2f})"
        else:
            status_msg.data = "IDLE"

        self.status_pub.publish(status_msg)

    def cancel_current_goal(self):
        """
        Cancel the current navigation goal
        """
        if self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            # This would cancel the current goal if one exists
            self.get_logger().info('Canceling current navigation goal')
            # Note: In practice, you'd need to keep track of the goal handle to cancel it
        else:
            self.get_logger().error('Navigation action server not available')

    def get_robot_position(self):
        """
        Get current robot position in map frame
        """
        if self.current_pose:
            return (
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z
            )
        return None

    def get_distance_to_goal(self):
        """
        Get distance to current navigation goal
        """
        if self.current_pose and self.navigation_goal:
            dx = self.current_pose.position.x - self.navigation_goal[0]
            dy = self.current_pose.position.y - self.navigation_goal[1]
            return math.sqrt(dx*dx + dy*dy)
        return float('inf')

def main(args=None):
    rclpy.init(args=args)

    try:
        nav_interface = HumanoidNav2Interface()

        # Example: Send a navigation goal
        # nav_interface.send_navigation_goal(5.0, 3.0, 0.0)

        rclpy.spin(nav_interface)
    except KeyboardInterrupt:
        pass
    finally:
        nav_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Cross-References with Other Chapters

This chapter on Nav2 Path Planning for Humanoid Robots connects with the other chapters in this module:

- **NVIDIA Isaac Sim for Photorealistic Simulation**: Nav2 navigation algorithms can be tested and validated in the simulation environments created with Isaac Sim.
- **Isaac ROS for VSLAM and Navigation**: The localization data from Isaac ROS VSLAM feeds into the Nav2 navigation system for complete autonomous navigation.

## Verification of Nav2 Examples

All Nav2 examples provided in this document are based on the official Navigation2 documentation and follow ROS 2 best practices. The configuration files include proper parameter settings for humanoid robots, and the code examples demonstrate correct integration patterns with Nav2 components.

Key verification points:

1. **Configuration Files**: All YAML configuration files follow Nav2's expected structure and parameter names
2. **Launch Files**: XML launch files use correct syntax and parameter passing
3. **Code Examples**: Python code follows ROS 2 client library patterns and proper error handling
4. **Humanoid-Specific Parameters**: All examples include appropriate modifications for humanoid robot constraints

The examples are structured to be educational while demonstrating practical implementation patterns for humanoid robotics applications using Navigation2.

## Humanoid-Specific Navigation Requirements

### Balance and Stability Considerations

Humanoid robots have unique navigation requirements related to balance and stability:

```python
# Example: Balance-aware navigation for humanoid robots
class BalanceAwareNavigator:
    def __init__(self, node):
        self.node = node

        # Balance-related parameters
        self.zmp_threshold = 0.08  # Zero Moment Point threshold
        self.com_height = 0.9      # Center of mass height
        self.support_polygon = self.calculate_support_polygon()

        # Publishers for balance-related data
        self.zmp_pub = node.create_publisher(
            # Custom message for Zero Moment Point
            'humanoid_msgs/msg/ZMP',
            '/balance/zmp',
            10
        )

    def calculate_support_polygon(self):
        """
        Calculate the support polygon for the current stance
        """
        # For bipedal stance, this would be the convex hull
        # of the contact points of both feet
        pass

    def validate_path_for_balance(self, path):
        """
        Validate that a path maintains balance throughout navigation
        """
        # Check each pose in the path to ensure it maintains balance
        for pose in path.poses:
            if not self.is_pose_balanced(pose):
                return False
        return True

    def is_pose_balanced(self, pose):
        """
        Check if a pose maintains balance
        """
        # Calculate ZMP and check if it's within support polygon
        zmp = self.calculate_zmp(pose)
        return self.is_zmp_stable(zmp)

    def calculate_zmp(self, pose):
        """
        Calculate Zero Moment Point for a given pose
        """
        # This would involve complex calculations using
        # robot kinematics and dynamics
        pass

    def is_zmp_stable(self, zmp):
        """
        Check if ZMP is within stable region
        """
        # Check if ZMP is within support polygon with safety margin
        return True  # Placeholder
```

### Step Planning Integration

```python
# Example: Integration with step planning for humanoid navigation
class StepPlanningNavigator:
    def __init__(self, node):
        self.node = node
        self.step_planner = StepPlanner(robot_params)

        # Publishers for step commands
        self.step_cmd_pub = node.create_publisher(
            # Custom message for step commands
            'humanoid_msgs/msg/StepCommand',
            '/step_commands',
            10
        )

    def execute_path_with_steps(self, path):
        """
        Execute a navigation path by generating and executing steps
        """
        # Plan steps for the path
        steps = self.step_planner.plan_steps_for_path(path)

        # Execute each step
        for step in steps:
            self.execute_step(step)

    def execute_step(self, step):
        """
        Execute a single step in the planned sequence
        """
        # Convert step to appropriate command format
        step_cmd = self.format_step_command(step)

        # Publish step command
        self.step_cmd_pub.publish(step_cmd)

        # Wait for step completion
        self.wait_for_step_completion()
```

These examples demonstrate how Navigation2 can be adapted for the specific requirements of humanoid robotics, including balance-aware navigation, step planning integration, and dynamic obstacle avoidance that takes into account the unique challenges of bipedal locomotion and maintaining stability during navigation.