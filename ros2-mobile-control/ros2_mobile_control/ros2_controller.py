import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose  # ✅ Import for feedback

class Ros2Controller(Node):
    def __init__(self):
        super().__init__('ros2_controller')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        print("✅ Ros2Controller initialized: Publisher to /turtle1/cmd_vel")

        self.mobile_interface = None  # Will be set from main.py

        # ✅ Create pose subscriber
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10
        )

    def set_mobile_interface(self, mobile_interface):
        self.mobile_interface = mobile_interface

    def _send_status(self, status):
        if self.mobile_interface:
            self.mobile_interface.send_ros_update({"status": status})

    def pose_callback(self, msg):
        pose_info = {
            "x": round(msg.x, 2),
            "y": round(msg.y, 2),
            "theta": round(msg.theta, 2)
        }
        print(f"[📡 Pose] {pose_info}")
        if self.mobile_interface:
            self.mobile_interface.send_ros_update({
                "status": f"Pose -> x: {pose_info['x']}, y: {pose_info['y']}, θ: {pose_info['theta']}"
            })

    def move_forward(self):
        print("➡️ Moving forward")
        twist = Twist()
        twist.linear.x = 1.0
        self.publisher.publish(twist)
        self._send_status("Moving forward")

    def move_left(self):
        print("⬅️ Turning left")
        twist = Twist()
        twist.angular.z = 1.0
        self.publisher.publish(twist)
        self._send_status("Turning left")

    def move_right(self):
        print("➡️ Turning right")
        twist = Twist()
        twist.angular.z = -1.0
        self.publisher.publish(twist)
        self._send_status("Turning right")

    def move_backward(self):
        print("⬇️ Moving backward")
        twist = Twist()
        twist.linear.x = -1.0
        self.publisher.publish(twist)
        self._send_status("Moving backward")

    def stop(self):
        print("⏹ Stopping")
        twist = Twist()
        self.publisher.publish(twist)
        self._send_status("Stopped")
