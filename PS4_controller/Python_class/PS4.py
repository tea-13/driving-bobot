from pyPS4Controller.controller import Controller
import serial
import time

class SerialMonitor():
    def __init__(self, _port : str, _baudrate : int):
        self.port = _port
        self.ser = None
        self.BAUDRATE = _baudrate


    def connect_serial(self) -> None:
        try:
            if self.ser is None or not self.ser.is_open:
                self.ser = serial.Serial(self.port, self.BAUDRATE, timeout=1.0)
                self.ser.reset_input_buffer()
            print(f'Connect to port: {self.port}')

        except Exception as e:
            print(f'Error during connection: {e}')

            if self.ser.is_open:
                self.ser.close()

            print('5 seconds wait')
            time.sleep(5)
            self.connect_serial()


    def send_msg(self, msg: str) -> None:
        try:
            self.connect_serial()

            print(msg)
            self.ser.write(str.encode(msg))	

        except KeyboardInterrupt:
            if self.ser.is_open:
                self.ser.close()

            exit('Programs close')


class RobotController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

        self.velocity2 = [0, 0]
        self.velocity4 = [0, 0, 0, 0]
        self.MAX_SPEED = 200

        self.ser = SerialMonitor("/dev/ttyUSB0", 115200)


    # disconnect/connect methods
    def connect():
        msg = f"s\r\n"
        self.ser.send_msg(msg)


    def disconnect():
        msg = f"s\r\n"
        self.ser.send_msg(msg)


    # Methods for controlling the machine as usual
    def on_L3_up(self, value):
        value //= 150
        if abs(value) < 10: value = 0
        self.velocity2[0] = -value
        self.velocity2[1] = -value
        self.m2_send_speed()


    def on_L3_down(self, value):
        value //= 150
        if abs(value) < 10: value = 0
        self.velocity2[0] = -value
        self.velocity2[1] = -value
        self.m2_send_speed()


    def on_L3_left(self, value):
        value //= 150
        if abs(value) < 10: value = 0
        self.velocity2[0] = -value
        self.velocity2[1] = value
        self.m2_send_speed()


    def on_L3_right(self, value):
        value //= 150
        if abs(value) < 10: value = 0
        self.velocity2[0] = -value
        self.velocity2[1] = value
        self.m2_send_speed()

    
    # Methods for controlling the machine as an omnidirectional
    def on_up_arrow_press(self):
        self.velocity2[0] = self.MAX_SPEED
        self.velocity2[1] = self.MAX_SPEED
        self.m2_send_speed()
    
    
    def on_down_arrow_press(self):
        self.velocity2[0] = -self.MAX_SPEED
        self.velocity2[1] = -self.MAX_SPEED
        self.m2_send_speed()
   
   
    def on_left_arrow_press(self):
        self.velocity4[0] = -self.MAX_SPEED
        self.velocity4[1] = self.MAX_SPEED
        self.velocity4[2] = self.MAX_SPEED
        self.velocity4[3] = -self.MAX_SPEED
        self.m4_send_speed()
    
    
    def on_right_arrow_press(self):
        self.velocity4[0] = self.MAX_SPEED
        self.velocity4[1] = -self.MAX_SPEED
        self.velocity4[2] = -self.MAX_SPEED
        self.velocity4[3] = self.MAX_SPEED
        self.m4_send_speed()
    
    def on_up_down_arrow_release(self):
        msg = f"s\r\n"
        self.ser.send_msg(msg)
    
    
    def on_left_right_arrow_release(self):
        self.on_up_down_arrow_release()
    

    # Methods for sending speed
    def m2_send_speed(self):
        msg = f"m2 {self.velocity2[0]} {self.velocity2[1]}\r\n"
        self.ser.send_msg(msg)


    def m4_send_speed(self):
        msg = f"m4 {self.velocity4[0]} {self.velocity4[1]} {self.velocity4[2]} {self.velocity4[3]}\r\n"
        self.ser.send_msg(msg)


if __name__ == "__main__":
    controller = RobotController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
