import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class CmdVelocity(Node):
    def __init__(self, _port: str, _baudrate: int):
        super().__init__('cmd_vel')
        
        # Constant
        self.MAX_SPEED = 2.5

        # Serial parametrs
        self.baud = _baudrate
        self.port = _port

        # Create Serial
        self.ser = serial.Serial(self.port, self.baud)

        # Create subscriber
        self.vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_callback,
            10)

        self.vel_sub  # prevent unused variable warning


    def send_msg(self, msg: str):
        try:
            if not self.ser.is_open:
                self.ser.open()
            
            self.ser.flush()
            self.ser.write(str.encode(msg))

        except Exception as e:
            self.get_logger().info(f'Is serial opend: {e}')
            if self.ser.is_open:
                self.ser.close() # close serial port if it was open

            self.ser = serial.Serial()
            self.get_logger().info(f'reset serial')
            self.ser.baudrate = self.baud
            self.ser.port = self.port


    def vel_callback(self, msg: Twist):
        sign = lambda x: 1 if x >=0 else -1

        _vel = msg.linear.x
        _ang = msg.angular.z

        vel_a = _vel - _ang
        vel_b = _vel + _ang

        motor_vel_a = sign(vel_a) * min(abs(vel_a), self.MAX_SPEED)
        motor_vel_b = sign(vel_b) * min(abs(vel_a), self.MAX_SPEED)

        message = f"m2 {int(-motor_vel_a*100)} {int(-motor_vel_b*100)}\r\n"

        self.send_msg(message)
        self.get_logger().info(message)


    def __del__(self):
        self.get_logger().info("Destructor is run")

        if self.ser.is_open:
            # Send a command to stop the motors and close the serial port.
            self.send_msg("s\r\n")
            self.ser.close()


def main(args=None):
    try:
        rclpy.init(args=args)
        cmd_vel = CmdVelocity("/dev/ttyUSB0", 115200)
        rclpy.spin(cmd_vel)
        cmd_vel.destroy_node()
        rclpy.shutdown()
    
    except KeyboardInterrupt as e:
        cmd_vel.destroy_node()
        rclpy.shutdown()
        exit("Exit: Ctrl+C")


if __name__ == '__main__':
    main()
    
