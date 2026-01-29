# ROS2 TOPICS

Create a ROS 2 Python package and first implement a number_publisher node that uses rclpy to publish a constant value of type example_interfaces/msg/Int64 on the /number topic using a timer, verifying it works with ros2 topic echo /number after checking the message structure with ros2 interface show example_interfaces/msg/Int64