"""
    A motor driver python program for a four-wheeled holonomic robot
"""

import math
import time
import rospy
import serial


from geometry_msgs.msg import Twist
from readchar import readkey, key


class MotorCommunication:
    """
    A class for holding motor communication function
    """

    def __init__(self) -> None:
        self.modem_device = "/dev/ttyUSB1"
        self.baud_rate = 9600
        self.timex = 3
        self.ser = None
        self.wheel_radius = 0.127
        self.robot_radius = 0.362
        self.w_wheel = []

    def check_conn(self):
        """
        check if te connection is available before sending the message
        """
        try:
            self.ser = serial.Serial(
                self.modem_device,
                self.baud_rate,
                timeout=self.timex,
                parity=serial.PARITY_EVEN,
            )
            print("Serial details params: ", self.ser)
        except ConnectionError:
            print("Could not connect to usb port")

    def initialize_driver(self):
        """
        initialize the driver with hex code
        """
        intialize_servo_on = [
            0x01,
            0x10,
            0x00,
            0x7C,
            0x00,
            0x02,
            0x04,
            0x00,
            0x00,
            0x00,
            0x01,
            0x35,
            0x1E,
        ]
        intialize_servo_on_2 = [
            0x02,
            0x10,
            0x00,
            0x7C,
            0x00,
            0x02,
            0x04,
            0x00,
            0x00,
            0x00,
            0x01,
            0x3A,
            0x5A,
        ]
        intialize_servo_on_3 = [
            0x03,
            0x10,
            0x00,
            0x7C,
            0x00,
            0x02,
            0x04,
            0x00,
            0x00,
            0x00,
            0x01,
            0x3E,
            0xA6,
        ]
        intialize_servo_on_4 = [
            0x04,
            0x10,
            0x00,
            0x7C,
            0x00,
            0x02,
            0x04,
            0x00,
            0x00,
            0x00,
            0x01,
            0x24,
            0xD2,
        ]

        try:
            self.ser.write(intialize_servo_on)
            time.sleep(0.05)
            self.ser.write(intialize_servo_on_2)
            time.sleep(0.05)
            self.ser.write(intialize_servo_on_3)
            time.sleep(0.05)
            self.ser.write(intialize_servo_on_4)
            time.sleep(0.05)
            print("Initialization complete")
            # # print(res)
            # response = self.ser.read(10)
            # response = response.hex(":")
            # # result = hex(int.from_bytes(res, byteorder='big'))
            # print(response)
        except ConnectionError:
            print("Initialization failed")
        time.sleep(1)

    def test_send_speed(self):
        """
        test case by sending various type of speed
        """
        speed_500rpm = [
            0x01,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            0xF4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0xEE,
            0x48,
        ]

        speed_1000rpm = [
            0x01,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0x87,
            0x36,
        ]

        speed_500rpm_2 = [
            0x02,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            0xF4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0x99,
            0x48,
        ]

        speed_1000rpm_2 = [
            0x02,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0xF0,
            0x36,
        ]

        speed_500rpm_3 = [
            0x03,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            0xF4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0xB5,
            0x88,
        ]

        speed_1000rpm_3 = [
            0x03,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0xDC,
            0xF6,
        ]

        speed_500rpm_4 = [
            0x04,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
            0xF4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0x77,
            0x48,
        ]

        speed_1000rpm_4 = [
            0x04,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0x1E,
            0x36,
        ]

        print("speed_500rpm")
        self.ser.write(speed_500rpm)
        time.sleep(5)

        print("speed_1000rpm")
        self.ser.write(speed_1000rpm)
        time.sleep(5)

        print("speed_500rpm_2")
        self.ser.write(speed_500rpm_2)
        time.sleep(5)

        print("speed_1000rpm_2")
        self.ser.write(speed_1000rpm_2)
        time.sleep(5)
        print("speed_500rpm_3")
        self.ser.write(speed_500rpm_3)
        time.sleep(5)

        print("speed_1000rpm_3")
        self.ser.write(speed_1000rpm_3)
        time.sleep(5)

        print("speed_500rpm_4")
        self.ser.write(speed_500rpm_4)
        time.sleep(5)

        print("speed_1000rpm_4")
        self.ser.write(speed_1000rpm_4)
        time.sleep(5)

    def show_message_in_hex(self, message):
        # show message representation in HEX format
        print(f"Combined Data (Hex): {', '.join(hex(byte) for byte in message)}")

    def set_speed(self, id_wheel, speed):
        """_summary_
        Args:
            id_wheel (_type_): the id_wheel of the motor driver or wheel (1-4)
            speed (_type_): The speed of the wheel in rpm

        Returns:
            _type_: message that will be sent to the motor driver
        """
        id_byte = self.id_to_byte(id_wheel)
        speed_byte = self.speed_to_byte_command(speed)
        message_before_speed = [
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x30,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
        message_after_speed = [
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
        ]
        pre_crc = id_byte + message_before_speed + speed_byte + message_after_speed
        crc_two_bytes = self.calculate_modbus_crc(pre_crc)
        final_message = pre_crc + list(crc_two_bytes)
        return final_message

    def calculate_modbus_crc(self, data):
        # convert the data into crc format

        crc = 0xFFFF  # Initial value for Modbus crc
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001  # Polynomial: 0x8005
                else:
                    crc >>= 1

        # Swap bytes to get little-endian result
        crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)
        return crc.to_bytes(2, byteorder="big")

    def id_to_byte(self, id_wheel):
        # convert motor id_wheel to byte, will be used on sending command to motor driver
        return list(int.to_bytes(id_wheel, 1, "big", signed=True))

    def speed_to_byte_command(self, speed):
        # convert motor id_wheel to byte, will be used on sending command to motor driver
        speed = int(speed)
        return list(int.to_bytes(speed, 4, "big", signed=True))

    def stop_operation(self):
        # a hex command to stop the motor completely, for every motor
        stop = [
            0x01,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0x46,
            0xB9,
        ]
        stop_2 = [
            0x02,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0x31,
            0xB9,
        ]
        stop_3 = [
            0x03,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0x1D,
            0x79,
        ]
        stop_4 = [
            0x04,
            0x10,
            0x00,
            0x5A,
            0x00,
            0x0E,
            0x1C,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x09,
            0xC4,
            0x00,
            0x00,
            0x03,
            0xE8,
            0x00,
            0x00,
            0x00,
            0x01,
            0xDF,
            0xB9,
        ]
        print("stop")
        self.ser.write(stop)
        time.sleep(0.1)
        self.ser.write(stop_2)
        time.sleep(0.1)
        self.ser.write(stop_3)
        time.sleep(0.1)
        self.ser.write(stop_4)
        time.sleep(0.1)
        time.sleep(3)

    def read_response(self, number_of_bit=8):
        # read the response for motor driver
        response = self.ser.read(number_of_bit)
        # # result = hex(int.from_bytes(res, byteorder='big'))
        response = response.hex(":")
        print(response)
        time.sleep(0.1)

    def vw_to_wheel_rpm(self, velocity, rotation_speed):
        """_summary_
            Converting speed to rpm value for each wheel, with offset pi/4
        Args:
            velocity: in m/s
            rotation_speed: in rad/s
        """

        v_x = velocity * math.cos(math.pi / 4)
        v_y = velocity * math.sin(math.pi / 4)

        v_wheel_1 = v_y + self.robot_radius * rotation_speed
        v_wheel_2 = v_x + self.robot_radius * rotation_speed
        v_wheel_3 = -v_y + self.robot_radius * rotation_speed
        v_wheel_4 = -v_x + self.robot_radius * rotation_speed

        w_wheel_1_rpm = (v_wheel_1 * 30) / (math.pi * self.wheel_radius)
        w_wheel_2_rpm = (v_wheel_2 * 30) / (math.pi * self.wheel_radius)
        w_wheel_3_rpm = (v_wheel_3 * 30) / (math.pi * self.wheel_radius)
        w_wheel_4_rpm = (v_wheel_4 * 30) / (math.pi * self.wheel_radius)

        self.w_wheel = [w_wheel_1_rpm, w_wheel_2_rpm, w_wheel_3_rpm, w_wheel_4_rpm]

    def vx_vy_w_to_wheel_rpm(self, vx, vy, rotation_speed):
        # kinematics of four-wheeled holonomic robot
        gear_reduction = 50
        alpha = math.pi / 4  # offset between vx axis and wheel

        v_wheel_1 = (
            -math.sin(alpha - math.pi / 2) * vx
            + math.cos(alpha - math.pi / 2) * vy
            + self.robot_radius * rotation_speed
        )
        v_wheel_2 = (
            -math.sin(alpha - math.pi) * vx
            + math.cos(alpha - math.pi) * vy
            + self.robot_radius * rotation_speed
        )
        v_wheel_3 = (
            -math.sin(alpha + math.pi / 2) * vx
            + math.cos(alpha + math.pi / 2) * vy
            + self.robot_radius * rotation_speed
        )
        v_wheel_4 = (
            -math.sin(alpha) * vx
            + math.cos(alpha) * vy
            + self.robot_radius * rotation_speed
        )

        w_wheel_1_rpm = (
            (v_wheel_1 * 30) / (math.pi * self.wheel_radius) * gear_reduction
        )
        w_wheel_2_rpm = (
            (v_wheel_2 * 30) / (math.pi * self.wheel_radius) * gear_reduction
        )
        w_wheel_3_rpm = (
            (v_wheel_3 * 30) / (math.pi * self.wheel_radius) * gear_reduction
        )
        w_wheel_4_rpm = (
            (v_wheel_4 * 30) / (math.pi * self.wheel_radius) * gear_reduction
        )

        w_wheel_1_rpm = int(w_wheel_1_rpm)
        w_wheel_2_rpm = int(w_wheel_2_rpm)
        w_wheel_3_rpm = int(w_wheel_3_rpm)
        w_wheel_4_rpm = int(w_wheel_4_rpm)

        self.w_wheel = [w_wheel_1_rpm, w_wheel_2_rpm, w_wheel_3_rpm, w_wheel_4_rpm]
        # self.w_wheel =  [500, 500, 500, 500]

    def send_speed(self):
        """
        sending the speed command to motor driver
        """
        for id_wheel in range(4):
            message = self.set_speed(id_wheel + 1, self.w_wheel[id_wheel])
            self.ser.write(message)
            time.sleep(0.1)

    def read_character(self):
        # a function to read the keyboard key and convert them to speed
        while True:
            key_pressed = readkey()
            if key_pressed == key.UP or key_pressed == "w" or key_pressed == "W":
                print("move forward")
                self.vx_vy_w_to_wheel_rpm(0.1, 0, 0)
                # print(handle.w_wheel)
                self.send_speed()
                time.sleep(0.1)
            elif key_pressed == key.DOWN or key_pressed == "s" or key_pressed == "S":
                print("move backward")
                handle.vx_vy_w_to_wheel_rpm(-0.1, 0, 0)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif key_pressed == key.LEFT or key_pressed == "a" or key_pressed == "A":
                print("move to the left")
                handle.vx_vy_w_to_wheel_rpm(0, 0.1, 0)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif key_pressed == key.RIGHT or key_pressed == "d" or key_pressed == "D":
                print("move to the right")
                handle.vx_vy_w_to_wheel_rpm(0, -0.1, 0)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif key_pressed == "q" or key_pressed == "Q":
                print("Rotate Counter Clockwise (CCW)")
                handle.vx_vy_w_to_wheel_rpm(0, 0, 0.1)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif key_pressed == "e" or key_pressed == "E":
                print("Rotate clockwise (CW)")
                handle.vx_vy_w_to_wheel_rpm(0, 0, -0.1)
                # print(handle.w_wheel)
                handle.send_speed()
                time.sleep(0.1)
            elif key_pressed == "r" or key_pressed == "R":
                print("Stopping")
                handle.stop_operation()
            elif key_pressed == key.ENTER:
                break

    def ros_node_subscriber_callbak_send_speed(self, data: Twist):
        # ros subscriber callback to send the speed
        self.vx_vy_w_to_wheel_rpm(data.linear.x, data.linear.y, data.angular.z)
        rospy.loginfo(
            f"vx = {data.linear.x} \t vy = {data.linear.y} \t w = {data.angular.z}"
        )
        rospy.loginfo(self.w_wheel)
        self.send_speed()


if __name__ == "__main__":
    handle = MotorCommunication()
    handle.check_conn()
    handle.initialize_driver()
    rospy.set_param("/teleop_initial_speed", 0.1)
    rospy.set_param("/teleop_initial_turn", 0.1)

    rospy.init_node("motor_node", anonymous=True)
    pub = rospy.Subscriber(
        "/cmd_vel", Twist, handle.ros_node_subscriber_callbak_send_speed
    )
    rospy.Rate(10)
    rospy.spin()
