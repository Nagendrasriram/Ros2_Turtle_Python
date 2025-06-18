import rclpy
from ros2_mobile_control.ros2_controller import Ros2Controller
from ros2_mobile_control.mobile_interface import MobileInterface

def main(args=None):
    rclpy.init(args=args)

    # Step 1: Initialize controller
    ros2_controller = Ros2Controller()

    # Step 2: Initialize mobile interface (Flask + SocketIO)
    mobile_interface = MobileInterface(ros2_controller)

    # Step 3: Set up bidirectional communication
    ros2_controller.set_mobile_interface(mobile_interface)

    # ✅ Already perfect: Flask runs in background
    mobile_interface.run_server()

    try:
        rclpy.spin(ros2_controller)  # ✅ ROS keeps listening to /pose
    except KeyboardInterrupt:
        print("❌ Keyboard interrupt received. Shutting down.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
