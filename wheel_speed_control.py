import serial
import struct
import time
import sys
import math
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from readchar import readkey, key

class MotorCommunication():
    def __init__(self) -> None:
        self.modem_device = "/dev/ttyUSB1"
        self.baud_rate = 9600
        self.timex = 3
        self.ser = None
        self.WHEEL_RADIUS = 0.127
        self.ROBOT_RADIUS = 0.362


    def check_conn(self):
        try:
            self.ser = serial.Serial(self.modem_device, self.baud_rate, timeout=self.timex, parity=serial.PARITY_EVEN)
            print("Serial details params: ", self.ser)
        except:
            print("Could not connect to usb port")
    
    def initialize_driver(self):
        # intialize_servo_ON = [0x01, 0x06, 0x05, 0x12, 0x00, 0x01, 0xE8, 0xC3]
        intialize_servo_ON = [0x01, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x35, 0x1E]
        intialize_servo_ON_2 = [0x02, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x3A, 0x5A]
        intialize_servo_ON_3 = [0x03, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x3E, 0xA6]
        intialize_servo_ON_4 = [0x04, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x24, 0xD2] 

        try:
            res = self.ser.write(intialize_servo_ON)
            time.sleep(0.05)
            res = self.ser.write(intialize_servo_ON_2)
            time.sleep(0.05)
            res = self.ser.write(intialize_servo_ON_3)
            time.sleep(0.05)
            res = self.ser.write(intialize_servo_ON_4)
            time.sleep(0.05)
            print("Initialization complete")
            # # print(res)
            # response = self.ser.read(10)
            # response = response.hex(":")
            # # result = hex(int.from_bytes(res, byteorder='big'))
            # print(response)
        except:
            print("Initialization failed")
        time.sleep(1)

    def s16(self, value):
        return -(value & 0x8000) | (value & 0x7fff)
       
    def test_send_speed(self):
        
        speed_500rpm = [0x01, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0xEE, 0x48]

        speed_1000rpm = [0x01, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0x87, 0x36]
        
        speed_500rpm_2 = [0x02, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0x99, 0x48]

        speed_1000rpm_2 = [0x02, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x36]
        
        speed_500rpm_3 = [0x03, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0xB5, 0x88]

        speed_1000rpm_3 = [0x03, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0xDC, 0xF6]
        
        speed_500rpm_4 = [0x04, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0x77, 0x48]

        speed_1000rpm_4 = [0x04, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01, 0x1E, 0x36]
        
        # print("TEST SEND SPEED")

        print("speed_500rpm")
        res = self.ser.write(speed_500rpm)
        time.sleep(5) 

        print("speed_1000rpm")
        res = self.ser.write(speed_1000rpm)
        time.sleep(5) 

        print("speed_500rpm_2")
        res = self.ser.write(speed_500rpm_2)
        time.sleep(5) 

        print("speed_1000rpm_2")
        res = self.ser.write(speed_1000rpm_2)
        time.sleep(5) 
        print("speed_500rpm_3")
        res = self.ser.write(speed_500rpm_3)
        time.sleep(5) 

        print("speed_1000rpm_3")
        res = self.ser.write(speed_1000rpm_3)
        time.sleep(5) 

        print("speed_500rpm_4")
        res = self.ser.write(speed_500rpm_4)
        time.sleep(5) 

        print("speed_1000rpm_4")
        res = self.ser.write(speed_1000rpm_4)
        time.sleep(5) 

    def show_message_in_hex(self, message):
        print(f"Combined Data (Hex): {', '.join(hex(byte) for byte in message)}")


    def set_speed(self, id, speed):
        """_summary_
        Args:
            id (_type_): the id of the motor driver or wheel (1-4)
            speed (_type_): The speed of the wheel in rpm

        Returns:
            _type_: message that will be sent to the motor driver
        """
        id_byte = self.id_to_byte(id)
        speed_byte = self.speed_to_byte_command(speed)
        message_before_speed = [0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00]
        message_after_speed = [0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01]
        pre_CRC = id_byte + message_before_speed + speed_byte + message_after_speed
        CRC_two_bytes = self.calculate_modbus_crc(pre_CRC)
        final_message = pre_CRC + list(CRC_two_bytes)
        return final_message

    def calculate_modbus_crc(self, data):
        crc = 0xFFFF  # Initial value for Modbus CRC
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001  # Polynomial: 0x8005
                else:
                    crc >>= 1

        # Swap bytes to get little-endian result
        crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)
        return crc.to_bytes(2, byteorder='big')
    
    def id_to_byte(self, id):
        return list(int.to_bytes(id, 1, "big", signed=True))

    def speed_to_byte_command(self, speed):
        speed = int(speed)
        return list(int.to_bytes(speed, 4, 'big', signed=True))

    def stop_operation(self):
        stop = [0x01, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x46, 0xB9] 
        stop_2 = [0x02, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x31, 0xB9]
        stop_3 = [0x03, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x1D, 0x79]
        stop_4 = [0x04, 0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0xDF, 0xB9]
        print("stop")
        res = self.ser.write(stop)
        time.sleep(0.1)
        res = self.ser.write(stop_2)
        time.sleep(0.1)
        res = self.ser.write(stop_3)
        time.sleep(0.1)
        res = self.ser.write(stop_4)
        time.sleep(0.1)
        time.sleep(3)       

    def read_response(self, numberOfBit=8):
        response = self.ser.read(numberOfBit)
        # # result = hex(int.from_bytes(res, byteorder='big'))
        response = response.hex(":")
        print(response)
        time.sleep(0.1)

    def calculate_modbus_crc(self, data):
        crc = 0xFFFF  # Initial value for Modbus CRC

        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001  # Polynomial: 0x8005
                else:
                    crc >>= 1

        # Swap bytes to get little-endian result
        crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)
        
        return crc.to_bytes(2, byteorder='big')
    
    def vw_to_wheel_rpm(self, velocity, rotation_speed):
        """_summary_

        Args:
            velocity: in m/s
            rotation_speed: in rad/s
        """

        offset = math.pi/4
        # angle_for_vy = math.pi/2
        v_x = velocity * math.cos(math.pi/4)
        v_y = velocity * math.sin(math.pi/4)
        
        v_wheel_1 = v_y + self.ROBOT_RADIUS * rotation_speed
        v_wheel_2 = v_x + self.ROBOT_RADIUS * rotation_speed
        v_wheel_3 = -v_y + self.ROBOT_RADIUS * rotation_speed
        v_wheel_4 = -v_x + self.ROBOT_RADIUS * rotation_speed

        w_wheel_1_rpm = (v_wheel_1 * 30) / (math.pi * self.WHEEL_RADIUS)
        w_wheel_2_rpm = (v_wheel_2 * 30) / (math.pi * self.WHEEL_RADIUS)
        w_wheel_3_rpm = (v_wheel_3 * 30) / (math.pi * self.WHEEL_RADIUS)
        w_wheel_4_rpm = (v_wheel_4 * 30) / (math.pi * self.WHEEL_RADIUS)

        self.w_wheel =  [w_wheel_1_rpm, w_wheel_2_rpm, w_wheel_3_rpm, w_wheel_4_rpm]
        
    def vx_vy_w_to_wheel_rpm(self, vx, vy, rotation_speed):
        gear_reduction = 50
        alpha = math.pi/4 #offset between vx axis and wheel 

        v_wheel_1 = -math.sin(alpha - math.pi/2) * vx + math.cos(alpha - math.pi/2) * vy + self.ROBOT_RADIUS * rotation_speed
        v_wheel_2 = -math.sin(alpha - math.pi) * vx + math.cos(alpha - math.pi) * vy + self.ROBOT_RADIUS * rotation_speed
        v_wheel_3 = -math.sin(alpha + math.pi/2) * vx + math.cos(alpha + math.pi/2) * vy + self.ROBOT_RADIUS * rotation_speed
        v_wheel_4 = -math.sin(alpha ) * vx + math.cos(alpha) * vy + self.ROBOT_RADIUS * rotation_speed
        
        w_wheel_1_rpm = (v_wheel_1 * 30) / (math.pi * self.WHEEL_RADIUS) / gear_reduction
        w_wheel_2_rpm = (v_wheel_2 * 30) / (math.pi * self.WHEEL_RADIUS) / gear_reduction
        w_wheel_3_rpm = (v_wheel_3 * 30) / (math.pi * self.WHEEL_RADIUS) / gear_reduction
        w_wheel_4_rpm = (v_wheel_4 * 30) / (math.pi * self.WHEEL_RADIUS) / gear_reduction

        w_wheel_1_rpm = int(w_wheel_1_rpm)
        w_wheel_2_rpm = int(w_wheel_2_rpm)
        w_wheel_3_rpm = int(w_wheel_3_rpm)
        w_wheel_4_rpm = int(w_wheel_4_rpm)


        self.w_wheel =  [w_wheel_1_rpm, w_wheel_2_rpm, w_wheel_3_rpm, w_wheel_4_rpm]
        # self.w_wheel =  [500, 500, 500, 500]

        

    def send_speed(self):
        for id in range(4):
            # print(f"{id+1} {self.w_wheel[id]}")
            message = self.set_speed(id+1, self.w_wheel[id])
            res = self.ser.write(message)
            # self.ser.flush()
            time.sleep(0.1)
            # print(message) 

    def read_character(self):
        while True:
            keyPressed = readkey()
            if keyPressed == key.UP or keyPressed == "w" or keyPressed == "W":
                print("move forward")
                self.vx_vy_w_to_wheel_rpm(10, 0, 0)
                # print(handle.w_wheel)
                self.send_speed()
                time.sleep(0.1)
            elif keyPressed == key.DOWN or keyPressed == "s" or keyPressed == "S" :
                print("move backward")
                handle.vx_vy_w_to_wheel_rpm(-10, 0, 0)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif keyPressed == key.LEFT or keyPressed == "a" or keyPressed == "A":
                print("move to the left")
                handle.vx_vy_w_to_wheel_rpm(0, 10, 0)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif keyPressed == key.RIGHT or keyPressed == "d" or keyPressed == "D":
                print("move to the right")
                handle.vx_vy_w_to_wheel_rpm(0, -10, 0)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif keyPressed == "q" or keyPressed == "Q":
                print("Rotate Counter Clockwise (CCW)")
                handle.vx_vy_w_to_wheel_rpm(0, 0, 10)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif keyPressed == "e" or keyPressed == "E":
                print("Rotate clockwise (CW)")
                handle.vx_vy_w_to_wheel_rpm(0, 0, -10)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif keyPressed == "r" or keyPressed == "R":
                print("Stopping")
                handle.stop_operation()
            elif keyPressed == key.ENTER:
                break
    
    def ros_node_subscriber_callbak_send_speed(self, data:Twist):
        self.vx_vy_w_to_wheel_rpm(data.linear.x, data.linear.y, data.angular.z)
        self.send_speed()

if __name__ == "__main__":
    handle = MotorCommunication()
    handle.check_conn()
    handle.initialize_driver()
    rospy.init_node("motor_node", anonymous=True)
    pub = rospy.Subscriber("/cmd_vel", Twist, handle.ros_node_subscriber_callbak_send_speed)
    
    #forward
    # handle.vx_vy_w_to_wheel_rpm(10, 0, 0)
    # print(handle.w_wheel)
    # handle.send_speed()-

    # # reverse
    # time.sleep(0.1)
    # handle.vx_vy_w_to_wheel_rpm(-10, 0, 0)
    # print(handle.w_wheel)
    # handle.send_speed()

    # message = handle.set_speed(4, 1000)
    # handle.show_message_in_hex(message)
    # handle.test_send_speed()
    # handle.send_speed()
    # handle.read_character()


    #test negative speed
    # message = handle.set_speed(1, 100)
    # res = handle.ser.write(message)
    try:
        handle.read_character()
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected")
    finally:
        handle.stop_operation()

