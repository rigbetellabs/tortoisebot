#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import math
import board
import adafruit_bno055

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Initialize the publisher
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # Initialize the timer for periodic publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_imu_data)

        # Initialize the sensor
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.get_logger().info("BNO055 IMU initialized")

        # Define sensor-specific offsets
        self.offsets_magnetometer = (53, -130, -477)
        self.offsets_gyroscope = (-2, -2, 1)
        self.offsets_accelerometer = (-59, -2, -22)

    def publish_imu_data(self):
        # Create an Imu message
        msg = Imu()

        # Populate the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'

        # Read orientation (quaternion)
        quaternion = self.sensor.quaternion
        if quaternion is not None and len(quaternion) == 4:
            try:
                msg.orientation = Quaternion(
                    x=float(quaternion[1]),
                    y=float(quaternion[2]),
                    z=float(quaternion[3]),
                    w=float(quaternion[0])
                )
                msg.orientation_covariance = [-1.0] * 9  # Replace with known covariance if available
            except (TypeError, ValueError):
                self.get_logger().warn("Quaternion data type mismatch")
        else:
            self.get_logger().warn("Failed to read quaternion data")

        # Read angular velocity and apply offsets
        gyro = self.sensor.gyro
        if gyro is not None:
            try:
                msg.angular_velocity = Vector3(
                    x=float(math.radians(gyro[0] - self.offsets_gyroscope[0])),
                    y=float(math.radians(gyro[1] - self.offsets_gyroscope[1])),
                    z=float(math.radians(gyro[2] - self.offsets_gyroscope[2]))
                )
                msg.angular_velocity_covariance = [-1.0] * 9  # Replace with known covariance if available
            except (TypeError, ValueError):
                self.get_logger().warn("Gyroscope data type mismatch")
        else:
            self.get_logger().warn("Failed to read gyroscope data")

        # Read linear acceleration and apply offsets
        accel = self.sensor.linear_acceleration
        if accel is not None:
            try:
                msg.linear_acceleration = Vector3(
                    x=float(accel[0] - self.offsets_accelerometer[0]),
                    y=float(accel[1] - self.offsets_accelerometer[1]),
                    z=float(accel[2] - self.offsets_accelerometer[2])
                )
                msg.linear_acceleration_covariance = [-1.0] * 9  # Replace with known covariance if available
            except (TypeError, ValueError):
                self.get_logger().warn("Linear acceleration data type mismatch")
        else:
            self.get_logger().warn("Failed to read linear acceleration data")

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().debug("Published IMU data")

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IMU publisher stopped cleanly")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()