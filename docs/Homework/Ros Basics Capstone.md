# Ros2 Turtle Catcher
**Hector Cornejo Morales**
**Augusto Mantilla Molina**

> Activity: The objective of this assignment is to design a reactive control system in ROS2 using the turtlesim simulator.

---

**The sistem consist of**  
- A spawner node that continuously creates turtles and publishes their information.
- A controller node that:
    - Monitors the position of **turtle1**
    - Receives a list of alive turtles
    - Computes which turtle is the closest
    - Moves **turtle1** toward the nearest turtle
    - Calls a service to remove (catch) the turtle when it reaches it  
The controller implements a proportional control strategy to guide turtle1 toward the nearest target at all times.

## 1) Turtle Controller 
## Controller Implementation
### Libraries
``` code
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import numpy as np
from turtle_catcher_interfaces.msg import TurtleArray
from turtle_catcher_interfaces.srv import CatchTurtle 
```  
**What does it do?**  
- **rclpy** → ROS2 Python client library 
- **Node** → Base class for creating ROS2 nodes
- **Twist** → Message used to control turtle velocity
- **Pose** → Message that provides turtle position and orientation
- **math** and **numpy** → Used for geometric calculations
- **TurtleArray** → Custom message that contains all alive turtles
- **CatchTurtle** → Custom service used to remove a turtle

---

### Class Definition and Constructor
``` code
class Turtle_Controller(Node):

    def __init__(self):
        super().__init__('turtle_controller')

        self.cmd_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.alive_sub = self.create_subscription(TurtleArray, "alive_turtles", self.alive_turtles_callback, 10)

        self.catch_client = self.create_client(CatchTurtle, "catch_turtle")

        while not self.catch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for catch_turtle service...")

        self.timer = self.create_timer(0.01, self.control_loop)

        self.current_pose = None
        self.target_turtle = []

        self.Kp_d = 1.5
        self.Kp_theta = 6.0

        self.get_logger().info("Your Turtlesim Controller is operational")
```

**What does it do?**  
- Initializes the node with the name **turtle_controller** 
- **Creates**
    - A publisher to control **turtle1**
    - A subscriber to read the pose of **turtle1**
    - A subscriber to receive all alive turtles
- Creates a service client to request turtle removal
- Waits until the **catch_turtle** service is available
- Creates a timer that executes the control loop every 0.01 seconds
- Defines proportional gains for:
 - Distance control (Kp_d)
 - Orientation control (Kp_theta)

---

### Pose Callback
``` code
def pose_callback(self, msg):
    self.current_pose = msg
```  
**What does it do?**  
- Stores the current position and orientation of **turtle1** 
- This data is later used for distance and angle calculations 

---

### Alive Turtle Callback
``` code
def alive_turtles_callback(self, msg):
    self.target_turtle = msg.turtles
```  
**What does it do?**  
- Receives the list of currently alive turtles
- Stores them in **self.target_turtle**
- This list is used to determine which turtle is closest

---

### Control Loop
``` code
def control_loop(self):

    if self.current_pose is None or len(self.target_turtle) == 0:
        return

    min_distance = float('inf')
    nearest_turtle = None

    for turtle in self.target_turtle:
        dx = turtle.x - self.current_pose.x
        dy = turtle.y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < min_distance:
            min_distance = distance
            nearest_turtle = turtle

    if nearest_turtle is None:
        return

    dx = nearest_turtle.x - self.current_pose.x
    dy = nearest_turtle.y - self.current_pose.y

    distance = math.sqrt(dx**2 + dy**2)

    theta_target = math.atan2(dy, dx)
    angle_error = theta_target - self.current_pose.theta
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

    cmd = Twist()

    if distance < 0.3:

        request = CatchTurtle.Request()
        request.name = nearest_turtle.name
        self.catch_client.call_async(request)

        self.get_logger().info(f"Capturando {nearest_turtle.name}")

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

    else:
        cmd.linear.x = self.Kp_d * distance
        cmd.angular.z = self.Kp_theta * angle_error

    self.cmd_publisher.publish(cmd)
```  
**What does it do?**  
- Verifies that pose and turtle data are available.  
- Computes the distance to each turtle. 
- Selects the turtle with the smallest distance.  
- Calculates:
    - Distance to the target
    - Desired heading angle
    - Angular error
- Applies proportional control to generate velocity commands.
- If the turtle is close enough (distance < 0.3):
    - Calls the service to remove it.
    - Stops movement.

---

### Control Operation
The controller uses a Proportional (P) **control strategy** to make a proportional control for the linear and angular velocitys.  
The angle error is normalized between −π and 𝜋 to ensure smooth rotation.
This creates a reactive pursuit behavior where **turtle1** always moves toward the nearest turtle.

---

### Full Controller Code
``` code
#!/usr/bin/env python3

# ROS2 Python client library
import rclpy
from rclpy.node import Node

# Message used to control turtle velocity
from geometry_msgs.msg import Twist

# Message that provides turtle position and orientation
from turtlesim.msg import Pose

# Math libraries for distance and angle calculations
import math
import numpy as np

# Custom message containing a list of alive turtles
from turtle_catcher_interfaces.msg import TurtleArray

# Custom service used to remove a turtle
from turtle_catcher_interfaces.srv import CatchTurtle 


class Turtle_Controller(Node):
    """
    Controller node that moves turtle1 toward the nearest turtle
    and requests its removal once it is close enough.
    """

    def __init__(self):
        # Initialize the node with the name 'turtle_controller'
        super().__init__('turtle_controller')

        # Publisher that sends velocity commands to turtle1
        self.cmd_publisher = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10
        )

        # Subscriber that receives the pose of turtle1
        self.pose_sub = self.create_subscription(
            Pose, "turtle1/pose", self.pose_callback, 10
        )

        # Subscriber that receives the list of all alive turtles
        self.alive_sub = self.create_subscription(
            TurtleArray, "alive_turtles", self.alive_turtles_callback, 10
        )

        # Service client used to request turtle removal
        self.catch_client = self.create_client(
            CatchTurtle, "catch_turtle"
        )

        # Wait until the service is available
        while not self.catch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for catch_turtle service...")

        # Timer that executes the control loop at 100 Hz
        self.timer = self.create_timer(0.01, self.control_loop)

        # Stores the current pose of turtle1
        self.current_pose = None

        # Stores the list of alive turtles
        self.target_turtle = []

        # Proportional control gains
        self.Kp_d = 1.5      # Distance gain
        self.Kp_theta = 6.0  # Angular gain

        self.get_logger().info("Your Turtlesim Controller is operational")

    def pose_callback(self, msg):
        """
        Callback that stores the current pose of turtle1.
        """
        self.current_pose = msg

    def alive_turtles_callback(self, msg):
        """
        Callback that receives and stores all alive turtles.
        """
        self.target_turtle = msg.turtles

    def control_loop(self):
        """
        Main control loop:
        - Finds the nearest turtle
        - Moves turtle1 toward it
        - Calls the service to remove it when close enough
        """

        # Exit if pose data or turtle list is not available
        if self.current_pose is None or len(self.target_turtle) == 0:
            return

        # Variables used to find the nearest turtle
        min_distance = float('inf')
        nearest_turtle = None

        # Iterate through all turtles to find the closest one
        for turtle in self.target_turtle:
            dx = turtle.x - self.current_pose.x
            dy = turtle.y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < min_distance:
                min_distance = distance
                nearest_turtle = turtle

        # Safety check
        if nearest_turtle is None:
            return

        # Compute distance and direction to the nearest turtle
        dx = nearest_turtle.x - self.current_pose.x
        dy = nearest_turtle.y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # Desired heading angle
        theta_target = math.atan2(dy, dx)

        # Angular error between current and desired orientation
        angle_error = theta_target - self.current_pose.theta

        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        # Create velocity command message
        cmd = Twist()

        # If turtle is close enough, request removal
        if distance < 0.3:

            request = CatchTurtle.Request()
            request.name = nearest_turtle.name
            self.catch_client.call_async(request)

            self.get_logger().info(f"Capturando {nearest_turtle.name}")

            # Stop movement after catching
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        else:
            # Proportional control for movement
            cmd.linear.x = self.Kp_d * distance
            cmd.angular.z = self.Kp_theta * angle_error

        # Publish velocity command
        self.cmd_publisher.publish(cmd)


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create controller node
    node = Turtle_Controller()

    # Keep the node running
    rclpy.spin(node)

    # Shutdown ROS2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## 2) Turtle Spawner
### Requiers Libraries
``` code
import rclpy
from rclpy.node import Node
import random
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtle_catcher_interfaces.msg import Turtle
from turtle_catcher_interfaces.msg import TurtleArray
from turtle_catcher_interfaces.srv import CatchTurtle
```  
**What does it do?**  
- Spawn → Built-in turtlesim service to create turtles 
- Kill → Built-in turtlesim service to remove turtles
- Turtle → Custom message for a single turtl
- TurtleArray → Custom message for multiple turtles
- CatchTurtle → Custom service used by the controller

---

### Class Definition and Initialization
``` code
class TurtleSpawner(Node):
```
This class defines the spawner node.

### Full Spawner Code
``` code
#!/usr/bin/env python3

# ROS2 Python client library
import rclpy
from rclpy.node import Node

# Used to generate random spawn positions
import random

# Built-in turtlesim services
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

# Custom messages
from turtle_catcher_interfaces.msg import Turtle
from turtle_catcher_interfaces.msg import TurtleArray

# Custom service used by the controller to request turtle removal
from turtle_catcher_interfaces.srv import CatchTurtle


class TurtleSpawner(Node):
    """
    This node is responsible for:
    - Spawning turtles at random positions
    - Keeping track of all alive turtles
    - Publishing the list of alive turtles
    - Providing a service to remove turtles when captured
    """

    def __init__(self):
        # Initialize the node with name 'turtle_spawner'
        super().__init__('turtle_spawner')

        # Client to call the turtlesim spawn service
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # Client to call the turtlesim kill service
        self.kill_client = self.create_client(Kill, '/kill')

        # Service server that allows the controller to request turtle removal
        self.catch_service = self.create_service(
            CatchTurtle,
            '/catch_turtle',
            self.catch_callback
        )

        # Publisher that sends the list of alive turtles
        self.alive_publisher = self.create_publisher(
            TurtleArray,
            '/alive_turtles',
            10
        )

        # Internal list that stores all currently alive turtles
        self.alive_turtles = []

        # Counter used to generate unique turtle names
        self.turtle_counter = 0

        # Timer that spawns a turtle every 0.5 seconds
        self.spawn_timer = self.create_timer(0.5, self.spawn_turtle)

        self.get_logger().info("Turtle Spawner iniciado")

    def spawn_turtle(self):
        """
        Periodically spawns a new turtle at a random position.
        """

        # Check if the spawn service is available
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Spawn service no disponible")
            return

        # Generate random coordinates inside valid turtlesim area
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)

        # Create spawn request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = f"enemy_{self.turtle_counter}"

        # Send asynchronous spawn request
        future = self.spawn_client.call_async(request)

        # Register callback to handle response
        future.add_done_callback(
            lambda future: self.spawn_callback(future, x, y)
        )

        # Increment counter for next turtle
        self.turtle_counter += 1

    def spawn_callback(self, future, x, y):
        """
        Executed when the spawn service returns a response.
        Adds the new turtle to the internal list and publishes it.
        """

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Error en spawn: {e}")
            return

        # Create custom Turtle message
        new_turtle = Turtle()
        new_turtle.x = x
        new_turtle.y = y
        new_turtle.theta = 0.0
        new_turtle.name = response.name

        # Add new turtle to alive list
        self.alive_turtles.append(new_turtle)

        # Publish updated list
        self.publish_alive_turtles()

        self.get_logger().info(f"Spawned {response.name}")

    def publish_alive_turtles(self):
        """
        Publishes the current list of alive turtles.
        """

        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_publisher.publish(msg)

    def catch_callback(self, request, response):
        """
        Service callback executed when the controller requests
        the removal of a turtle.
        """

        turtle_name = request.name
        self.get_logger().info(f"Intentando eliminar {turtle_name}")

        # Check if kill service is available
        if not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Kill service no disponible")
            response.success = False
            return response

        # Create kill request
        kill_request = Kill.Request()
        kill_request.name = turtle_name

        # Send asynchronous kill request
        self.kill_client.call_async(kill_request)

        # Remove turtle from internal list
        self.alive_turtles = [
            turtle for turtle in self.alive_turtles
            if turtle.name != turtle_name
        ]

        # Publish updated list
        self.publish_alive_turtles()

        self.get_logger().info(f"{turtle_name} eliminada")

        response.success = True
        return response


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create spawner node
    node = TurtleSpawner()

    # Keep node running
    rclpy.spin(node)

    # Shutdown ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Results

<img src="../recursos/imgs/result_turtle.gif" alt="Turtle catcher result" width="800">