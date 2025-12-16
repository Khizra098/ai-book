---
title: Sensor Simulation & Validation
---

# Sensor Simulation & Validation

## Understanding Sensor Simulation for Humanoid Robots

Sensor simulation is a critical component of digital twin environments for humanoid robots. Accurate simulation of sensors allows for realistic perception system development, testing, and validation without the need for expensive physical hardware. This chapter covers the simulation of key sensors used in humanoid robotics: LiDAR, depth cameras, and IMU sensors.

### The Importance of Sensor Simulation

Sensor simulation enables:
- Development and testing of perception algorithms in a safe environment
- Validation of robot behavior under various sensor conditions
- Generation of training data for machine learning models
- Testing of sensor fusion algorithms
- Evaluation of robot performance in diverse scenarios

## LiDAR Simulation Techniques

### LiDAR Fundamentals

LiDAR (Light Detection and Ranging) sensors emit laser pulses and measure the time it takes for the light to return after reflecting off objects. This provides accurate 3D distance measurements that are crucial for navigation, mapping, and obstacle detection.

### Simulating LiDAR in Gazebo

Gazebo provides realistic LiDAR simulation through ray sensors:

```xml
<sensor name="lidar_sensor" type="ray">
  <pose>0 0 0.5 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
        <max_angle>1.570796</max_angle>    <!-- 90 degrees -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
    <topicName>/laser_scan</topicName>
    <frameName>lidar_link</frameName>
  </plugin>
</sensor>
```

### LiDAR Configuration Parameters

Key parameters for LiDAR simulation:
- **Samples**: Number of rays in the horizontal scan
- **Range**: Minimum and maximum detection distance
- **Field of View**: Angular coverage of the sensor
- **Resolution**: Angular resolution of the sensor
- **Update Rate**: Frequency of sensor readings

### Noise Modeling

Real LiDAR sensors have noise characteristics that should be simulated:

```xml
<sensor name="lidar_sensor" type="ray">
  <!-- ... previous configuration ... -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
  </noise>
</sensor>
```

## Depth Camera and IMU Simulation

### Depth Camera Simulation

Depth cameras provide both RGB images and depth information, making them valuable for 3D perception:

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <baseline>0.2</baseline>
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>depth_camera</cameraName>
    <imageTopicName>/rgb/image_raw</imageTopicName>
    <depthImageTopicName>/depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>/depth/points</pointCloudTopicName>
    <cameraInfoTopicName>/rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>/depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>depth_camera_optical_frame</frameName>
    <pointCloudCutoff>0.1</pointCloudCutoff>
    <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <CxPrime>0.0</CxPrime>
    <Cx>0.0</Cx>
    <Cy>0.0</Cy>
    <focalLength>0.0</focalLength>
  </plugin>
</sensor>
```

### IMU Simulation

Inertial Measurement Units (IMUs) provide orientation and acceleration data:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- 0.1 degree/s std dev -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- 0.017 m/s² std dev -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <topicName>/imu/data</topicName>
    <bodyName>imu_link</bodyName>
    <frameName>imu_link</frameName>
    <serviceName>/default_imu_service</serviceName>
    <gaussianNoise>0.0017</gaussianNoise>
    <updateRate>100.0</updateRate>
  </plugin>
</sensor>
```

## Sensor Fusion Concepts

### Combining Multiple Sensors

Sensor fusion combines data from multiple sensors to improve perception accuracy:

```cpp
// Example sensor fusion for position estimation
class SensorFusion {
private:
    ros::Subscriber lidar_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber camera_sub;

    // Extended Kalman Filter for state estimation
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;

public:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Process LiDAR data
        updateWithLidar(scan);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
        // Process IMU data
        updateWithImu(imu);
    }

    void cameraCallback(const sensor_msgs::Image::ConstPtr& image) {
        // Process camera data
        updateWithCamera(image);
    }

    void updateWithLidar(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Implement LiDAR-based state update
        // Fuse with current state estimate
    }

    void updateWithImu(const sensor_msgs::Imu::ConstPtr& imu) {
        // Implement IMU-based state update
        // Integrate acceleration and angular velocity
    }

    void updateWithCamera(const sensor_msgs::Image::ConstPtr& image) {
        // Implement camera-based state update
        // Use visual features for position estimation
    }
};
```

### Kalman Filters for Sensor Fusion

Kalman filters provide optimal state estimation by combining sensor measurements with a dynamic model:

1. **Prediction Step**: Estimate state based on motion model
2. **Update Step**: Correct estimate using sensor measurements
3. **Covariance Update**: Adjust uncertainty based on measurement quality

## Validation Techniques for Sensor Data

### Comparing Simulated vs. Real Data

Validation involves comparing simulated sensor data with real sensor characteristics:

```python
import numpy as np
import matplotlib.pyplot as plt

def validate_lidar_simulation(simulated_data, real_data, tolerance=0.05):
    """
    Validate LiDAR simulation against real data
    tolerance: acceptable percentage difference (5% = 0.05)
    """
    # Calculate statistics for both datasets
    sim_mean = np.mean(simulated_data)
    sim_std = np.std(simulated_data)
    real_mean = np.mean(real_data)
    real_std = np.std(real_data)

    # Check if means are within tolerance
    mean_diff = abs(sim_mean - real_mean) / real_mean
    std_diff = abs(sim_std - real_std) / real_std

    print(f"Mean difference: {mean_diff:.3f} (tolerance: {tolerance})")
    print(f"Std dev difference: {std_diff:.3f} (tolerance: {tolerance})")

    # Return True if within tolerance
    return mean_diff <= tolerance and std_diff <= tolerance

def validate_imu_simulation(sim_imu_data, real_imu_data):
    """
    Validate IMU simulation by checking noise characteristics
    """
    # Analyze noise properties
    sim_accel_noise = np.std(sim_imu_data.linear_acceleration)
    real_accel_noise = np.std(real_imu_data.linear_acceleration)

    sim_gyro_noise = np.std(sim_imu_data.angular_velocity)
    real_gyro_noise = np.std(real_imu_data.angular_velocity)

    # Check if noise characteristics match
    accel_noise_ratio = sim_accel_noise / real_accel_noise
    gyro_noise_ratio = sim_gyro_noise / real_gyro_noise

    print(f"Acceleration noise ratio: {accel_noise_ratio:.3f}")
    print(f"Gyroscope noise ratio: {gyro_noise_ratio:.3f}")

    # Acceptable range: 0.8 to 1.2 (20% tolerance)
    return 0.8 <= accel_noise_ratio <= 1.2 and 0.8 <= gyro_noise_ratio <= 1.2
```

### Point Cloud Validation

For LiDAR sensors, validate point cloud quality:

```python
def validate_point_cloud(sim_points, real_points, tolerance=0.05):
    """
    Validate LiDAR point cloud simulation
    """
    # Check point density
    sim_density = len(sim_points) / calculate_volume(sim_points)
    real_density = len(real_points) / calculate_volume(real_points)

    # Check geometric accuracy
    distances = []
    for sim_point, real_point in zip(sim_points[:len(real_points)], real_points):
        dist = np.linalg.norm(np.array(sim_point) - np.array(real_point))
        distances.append(dist)

    avg_distance = np.mean(distances)
    max_distance = np.max(distances)

    print(f"Average point distance: {avg_distance:.3f}m")
    print(f"Max point distance: {max_distance:.3f}m")
    print(f"Point cloud accuracy: {avg_distance < tolerance}")

    return avg_distance < tolerance
```

## Runnable Configuration Examples

### Complete Sensor Configuration

Here's a complete example of a humanoid robot with multiple sensors:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_sensors">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>

  <!-- Head with sensors -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="torso_head_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- IMU in head -->
  <sensor name="head_imu" type="imu">
    <pose>0 0 0 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <parent>head</parent>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0017</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0017</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0017</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></z>
      </linear_acceleration>
    </imu>
  </sensor>

  <!-- Depth camera in head -->
  <sensor name="head_camera" type="depth">
    <pose>0.05 0 0 0 0 0</pose>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30.0</update_rate>
    <parent>head</parent>
  </sensor>

  <!-- LiDAR on torso -->
  <sensor name="torso_lidar" type="ray">
    <pose>0 0 0.1 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>20.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <always_on>true</always_on>
    <update_rate>10.0</update_rate>
    <parent>torso</parent>
  </sensor>
</robot>
```

## Ensuring Accuracy Within Tolerance

### Sensor Calibration

Proper calibration ensures sensor simulation accuracy:

1. **Intrinsic Calibration**: Camera focal length, distortion parameters
2. **Extrinsic Calibration**: Sensor positions and orientations relative to robot
3. **Noise Calibration**: Actual noise characteristics from real sensors

### Validation Workflow

1. **Collect real sensor data** from physical robot
2. **Configure simulation** with same parameters
3. **Generate simulated data** under identical conditions
4. **Compare datasets** using statistical methods
5. **Adjust simulation parameters** to minimize differences
6. **Repeat validation** until accuracy requirements are met

## Humanoid-Specific Sensor Applications

### Perception for Humanoid Locomotion

Humanoid robots require specialized sensor configurations:

- **Balance sensors**: IMUs for maintaining stability
- **Obstacle detection**: LiDAR and cameras for navigation
- **Foot contact**: Force/torque sensors for gait control
- **Upper body sensing**: For manipulation tasks

### Multi-Sensor Integration

Humanoid robots benefit from multiple coordinated sensors:

- **Head sensors**: Vision and orientation for navigation
- **Body sensors**: IMUs for balance and motion
- **Limb sensors**: Force/torque for manipulation
- **Environmental sensors**: LiDAR for mapping and obstacle avoidance

## Summary

Sensor simulation is essential for creating realistic digital twins of humanoid robots. By accurately simulating LiDAR, depth cameras, and IMU sensors with proper noise models and validation techniques, developers can create trustworthy simulation environments that closely match real-world behavior. Proper validation against real sensor data ensures that simulation results are applicable to physical robots.