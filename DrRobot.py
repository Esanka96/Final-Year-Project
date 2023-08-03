import socket
import time
from socket import *

from Robot_Config import *


class DrRobot:
    send_freq: int = 60  # Hz

    def __init__(self) -> None:
        super().__init__()
        self.client_socket: socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)
        self.client_socket.connect((RobotIP, Port1))
       
        
    def ping(self) -> None:
        self.send_cmd("PING")

    # Functions to Drive the Robot
    
    def go_forward(self, power1: int, power2: int, seconds: float) -> None:
        # print("Forward")
        self.send_cmd("MMW !M {0} -{1}".format(str(power1), str(power2)))
        for i in range(0, seconds * self.send_freq):
            self.ping()
        self.stop()
        
    def go_backward(self, power1: int,power2: int, seconds: int) -> None:
        # print("Backward")
        self.send_cmd("MMW !M -{0} {1}".format(str(power1), str(power2)))
        for i in range(0, (seconds * self.send_freq) - 1):
            self.ping()
        self.stop()
        
    def turn_left(self, power: int, seconds: int) -> None:
        # print("Left")
        self.send_cmd("MMW !M -{0} -{1}".format(str(power), str(power)))
        for i in range(0, (seconds * self.send_freq) - 1):
            self.ping()
        self.stop()
        
    def turn_right(self, power: int, seconds: int) -> None:
        # print("Right")
        self.send_cmd("MMW !M {0} {1}".format(str(power), str(power)))
        for i in range(0, (seconds * self.send_freq) - 1):
            self.ping()
        self.stop()
        
    def stop(self) -> None:
        self.send_cmd("MMW !M 0 0")
        
    def emergency_stop(self) -> None:
        self.send_cmd("MMW !M 0 0")
        self.send_cmd("MMW !EX")
        
    def emergency_stop_release(self) -> None:
        self.send_cmd("MMW !MG")
        
    def send_cmd(self, cmd: str) -> None:
        data = bytes(cmd, 'utf-8') + b'\r\n'
        self.client_socket.sendall(data)
        time.sleep(0.001/self.send_freq)

    # Functions to Read Sensor Data
    
    def read(self):
        sockFile = self.client_socket.makefile()

        while True:
            message = sockFile.readline().rstrip()
            self.process(message)

    def process(self, message):
        if (message.startswith('MM')):
            self.process_motor_message(message)
     
    def process_motor_message(self, message):
        
        # param message: MMX Y=Z   (X = motor driver board id (0=front) or (1=rear) , Y = parameter name, Z = parameter
        driver_board_id = int(message[2])

        if (message[4:].startswith('C=')):
            self.process_motor_encoder_position_count(driver_board_id, message[6:])
        elif (message[4:].startswith('S=')):
            self.process_motor_encoder_velocity(driver_board_id, message[6:])
        elif (message[4:].startswith('CR=')):
            self.process_motor_encoder_position_count_relative(driver_board_id, message[7:])
        
    def process_motor_encoder_position_count(self, driver_board_id, param):
        values = param.split(":")
        p = ""
        if driver_board_id == 0:
            p = "front"
        elif driver_board_id == 1:
            p = "rear"
        print("Encoder position count left {0} = {1}".format(p, values[0]))
        print("Encoder position count right {0} = {1}".format(p, values[1]))
        
    def process_motor_encoder_position_count_relative(self, driver_board_id, param):
        values = param.split(":")
        p = ""
        if driver_board_id == 0:
            p = "front"
        elif driver_board_id == 1:
            p = "rear"
        print("Encoder position count relative left {0} = {1}".format(p, values[0]))
        print("Encoder position count relative right {0} = {1}".format(p, values[1]))
        
    def process_motor_encoder_velocity(self, driver_board_id, param) :
        values = param.split(":")
        p = ""
        if driver_board_id == 0:
            p = "front"
        elif driver_board_id == 1:
            p = "rear"
        print("Encoder velocity  {0} = {1} RPM".format(p, values[0]))
        print("Encoder velocity  {0} = {1} RPM".format(p, values[1]))
        
    def close_connection(self):
        self.client_socket.close()
        
