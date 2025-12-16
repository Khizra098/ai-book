---
sidebar_label: 'ROS 2 Communication Model'
sidebar_position: 2
---

# ROS 2 Communication Model

## Nodes Concept and Implementation

In ROS 2, a node is a fundamental unit of computation that performs specific tasks. Nodes are processes that perform computation and communicate with other nodes through messages. Each node typically performs a specific function and can be combined with other nodes to create complex robotic behaviors.

### Creating a Node

To create a node in ROS 2, you typically inherit from the `rclpy.Node` class in Python or use the appropriate base class in C++. Here's a basic example of a node implementation:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # Initialize node components here
        self.get_logger().info('Minimal node initialized')

def main(args=None):
    rclpy.init(args=args)

    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

ROS 2 nodes have a well-defined lifecycle that includes:
- **Unconfigured**: Initial state after creation
- **Inactive**: Configured but not executing
- **Active**: Fully operational and executing
- **Finalized**: Cleaned up and ready for destruction

## Topics and Publisher-Subscriber Pattern

The publisher-subscriber pattern is the most common communication pattern in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics without direct knowledge of each other.

### Implementing a Publisher

Here's how to implement a publisher in Python using rclpy:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Implementing a Subscriber

And here's how to implement a subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services and Client-Server Pattern

Services provide a request-response communication pattern in ROS 2. A service client sends a request to a service server, which processes the request and returns a response.

### Implementing a Service Server

Here's how to implement a service server:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Implementing a Service Client

And here's how to implement a service client:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (1, 2, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions for Long-Running Tasks

Actions are used for long-running tasks that require feedback and status updates. They provide a more robust communication pattern than services for tasks that take significant time to complete.

### Implementing an Action Server

Here's how to implement an action server:

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = MinimalActionServer()

    try:
        rclpy.spin(minimal_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Basic rclpy-based Agent Controller Flow

An agent controller in ROS 2 typically follows this flow:

1. **Initialization**: Create the node and set up all necessary components
2. **Setup**: Create publishers, subscribers, services, or actions as needed
3. **Execution Loop**: Either use callbacks for asynchronous processing or a main loop for synchronous processing
4. **Cleanup**: Properly shut down all components

Here's a complete example of a simple agent controller:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initialize control variables
        self.obstacle_detected = False
        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg):
        # Simple obstacle detection using laser scan
        if len(msg.ranges) > 0:
            min_range = min(msg.ranges)
            self.obstacle_detected = min_range < 1.0  # 1 meter threshold

    def control_loop(self):
        cmd_vel = Twist()

        if self.obstacle_detected:
            # Stop or turn when obstacle detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right
        else:
            # Move forward when no obstacle
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Communication Patterns

In humanoid robotics, communication patterns often involve:

1. **Sensor Integration**: Multiple sensors (IMU, cameras, force sensors) providing data to perception nodes
2. **Motion Control**: Coordinated control of multiple joints for locomotion and manipulation
3. **Behavior Coordination**: Higher-level nodes orchestrating complex behaviors
4. **Human-Robot Interaction**: Nodes handling speech, gesture, and social interaction

These patterns require careful consideration of timing, reliability, and real-time constraints that are specific to humanoid applications.

## Summary

ROS 2 communication patterns provide the foundation for building complex robotic systems. The publisher-subscriber pattern enables decoupled communication, services provide request-response interactions, and actions handle long-running tasks with feedback. Understanding these patterns is essential for developing effective humanoid robot controllers.