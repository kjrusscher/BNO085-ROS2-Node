import time
from math import atan2, sqrt
from math import pi as PI

from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

linear_accel = [0,0,0] # x, y, z  (in m/s^2)
gyro = [0,0,0] # x, y, z  (in rad/s)

class BNO085_Publisher(Node):
    def __init__(self):
        super().__init__('BNO085_Publisher')
        # create the publisher for the IMU data
        self.imu_data_publisher = self.create_publisher(
            Imu, # ROS Message
            'IMU_Data',  # Topic
            10)
        
        # IMU sensor (BNO085)
        self.imu = None
        self.init_sensor()

        # create timer for reading and publishing data (@ rate of ~4hz)
        self.read_send_timer = self.create_timer(0.25, self.read_and_send_imu_data)

    def init_sensor(self):
        i2c = I2C(3)
        try:
            self.imu = BNO08X_I2C(i2c)
        except:
            self.get_logger().error('Failed to connect to BNO085 via I2C...')
            raise Exception('Failed to connect to BNO085 via I2')
            

        # enable the reports from the IMU
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION) # Linear acceleration data
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE) # For Angular Velocity data

        time.sleep(0.5) # Make sure we the IMU is initialized

    def read_and_send_imu_data(self):
        # get the Angular Velocity (gryo data) of the robot
        gyro[0], gyro[1], gyro[2] = self.imu.gyro 
        # get the Linear Acceleration of the robot
        linear_accel[0], linear_accel[1], linear_accel[2] = self.imu.linear_acceleration  

        #create messages to publish
        imu_data_msg = Imu()
        imu_data_msg.header = Header()
        imu_data_msg.header.stamp = self.get_clock().now().to_msg()
        imu_data_msg.header.frame_id = "bno085_frame"

        # TODO: Double check that this is true
        # IMU X right, Y forward, Z up
        # ROS Y left, X forward, Z up
        imu_data_msg.angular_velocity.x = gyro[0]
        imu_data_msg.angular_velocity.y = gyro[1]
        imu_data_msg.angular_velocity.z = gyro[2]
        imu_data_msg.linear_acceleration.x = linear_accel[0]
        imu_data_msg.linear_acceleration.y = linear_accel[1]
        imu_data_msg.linear_acceleration.z = linear_accel[2]
        imu_data_msg.orientation.x = 0.0
        imu_data_msg.orientation.y = 0.0
        imu_data_msg.orientation.z = 0.0
        imu_data_msg.orientation.w = 0.0
        
        #TODO: Look into getting the IMU's Covariance values
        # Following the recommendation here: https://robotics.stackexchange.com/questions/22756/what-would-be-a-way-to-estimate-imu-noise-covariance-matrix
        # we set the covariance values to a small values (just in case they are needed) on the diagonal
        # For now the covariances are set to -1 as sign that they can be ignored.
        imu_data_msg.orientation_covariance[0] = -1
        # imu_data_msg.orientation_covariance[4] = 0.01
        # imu_data_msg.orientation_covariance[8] = 0.01
        imu_data_msg.angular_velocity_covariance[0] = -1
        # imu_data_msg.angular_velocity_covariance[4] = 0.01
        # imu_data_msg.angular_velocity_covariance[8] = 0.01
        imu_data_msg.linear_acceleration_covariance[0] = -1
        # imu_data_msg.linear_acceleration_covariance[4] = 0.01
        # imu_data_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_data_publisher.publish(imu_data_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        bno_publisher = BNO085_Publisher()
        rclpy.spin(bno_publisher)
        bno_publisher.destroy_node()
    except Exception as e: 
        print(orientation_quat)
        print("------------------")
        print(traceback.format_exec())
        #print(e)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
