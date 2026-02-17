# Activity - ROS 2 Services

## Introduction
In distributed systems developed with ROS 2, communication between nodes can be achieved through different mechanisms, with topics and services being two of the most fundamental. While topics enable continuous, asynchronous communication based on the publisher–subscriber model, services introduce a synchronous request–response interaction.

This activity focuses on deepening the understanding of ROS 2 services within an existing topic-based system. Starting from a node that publishes numbers and another that accumulates them into a counter, a new capability is introduced: resetting the accumulated value through a service call. This approach highlights how on-demand interactions can be integrated into a distributed architecture, reinforcing modular design and controlled system behavior.

### Publisher Code
``` codigo
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounter(Node):

    def __init__(self):
        super().__init__('number_counter')
        self.counter = 0
        self.subscription = self.create_subscription(
            Int64,
            '/number',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Int64,
            '/number_count',
            10
        )
        self.get_logger().info('Number counter working...')

    def listener_callback(self, msg):
        self.counter += msg.data
        out_msg = Int64()
        out_msg.data = self.counter
        self.publisher_.publish(out_msg)
        self.get_logger().info(
            f'Recibido: {msg.data} | Total: {self.counter}'
        )

def main(args=None):
    rclpy.init(args=args)
    nodito = NumberCounter()
    rclpy.spin(nodito)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This code implements a node named number_counter, whose primary function is to receive numerical values, accumulate them, and publish the updated result.

The node subscribes to the /number topic, where it receives messages of type Int64. Each time a new message arrives, the listener_callback method is automatically triggered, adding the received value to an internal variable that acts as an accumulative counter. The updated total is then published on the /number_count topic, making the information available to other nodes in the system.

### Suscriber Code
``` codigo
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounter(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter = 0
        self.subscription = self.create_subscription(
            Int64,
            '/number',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Int64,
            '/number_count',
            10
        )
        self.service = self.create_service(
            SetBool,
            '/reset_counter',
            self.reset_callback
        )
        self.get_logger().info('Number counter working...')

    def listener_callback(self, msg):
        self.counter += msg.data
        out_msg = Int64()
        out_msg.data = self.counter
        self.publisher_.publish(out_msg)
        self.get_logger().info(
            f'Recibido: {msg.data} | Total: {self.counter}'
        )

    def reset_callback(self, request, response):
        if request.data:
            self.counter = 0
            response.success = True
            response.message = "Counter reset to zero"
            self.get_logger().info("Counter has been reset!")
        else:
            response.success = False
            response.message = "Reset not requested"
        return response


def main(args=None):
    rclpy.init(args=args)
    nodito = NumberCounter()
    rclpy.spin(nodito)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
This code implements a node named number_counter that integrates both the publisher–subscriber model and the client–server model through services.

On one hand, the node subscribes to the /number topic, receiving Int64 messages. Each time a new value arrives, the listener_callback method is automatically triggered, adding the received data to an internal accumulative counter. The updated total is then published on the /number_count topic, maintaining a continuous reactive processing flow.

On the other hand, the node includes a service server named /reset_counter, using the SetBool service type. Through the reset_callback method, external requests can directly modify the node’s internal state. If the request contains a true boolean value, the counter is reset to zero and a confirmation response is returned. If the value is false, the state remains unchanged and the response indicates that no reset was performed.

## Results
In the following image you can see the code working in the terminals, one is sending a message, the other one is reciving the message sent by the other terminal and the last one resets the counter to zero.

![Diagrama del sistema](../recursos/imgs/reset.jpg)