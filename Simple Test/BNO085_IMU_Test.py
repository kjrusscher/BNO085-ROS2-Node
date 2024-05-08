import time
from math import atan2, sqrt
from math import pi as PI
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

def main(args=None):
    i2c = I2C(3)
    try:
        imu = BNO08X_I2C(i2c)
    except:
        raise Exception('Failed to connect to BNO085 via I2')

    # enable the reports from the IMU
    imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION) # Linear acceleration data
    imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE) # For Angular Velocity data

    linear_accel = [0,0,0] # x, y, z  (in m/s^2)
    gyro = [0,0,0] # x, y, z  (in deg/s)

    while True:
        time.sleep(0.1)
        
        print("Gyro:")
        gyro[0], gyro[1], gyro[2] = imu.gyro  # pylint:disable=no-member
        print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro[0], gyro[1], gyro[2]))
        print("")

        print("Linear Acceleration:")
        linear_accel[0], linear_accel[1], linear_accel[2] = imu.linear_acceleration  # pylint:disable=no-member
        print(
            "X: %0.6f  Y: %0.6f Z: %0.6f m/s^2"
            % (linear_accel[0], linear_accel[1], linear_accel[2])
        )
        print("")
        
        
        time.sleep(0.2)

if __name__ == "__main__":
    main()
