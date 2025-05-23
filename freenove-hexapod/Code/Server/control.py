# -*- coding: utf-8 -*-
import time
import math
import copy
import threading
import numpy as np
from gpiozero import OutputDevice

from pid import Incremental_PID
from command import COMMAND as cmd
from imu import IMU
from servo import Servo

# When the Control is initialized, it will set up the IMU and Servo classes, as well as other necessary components.
# It will also initialize the leg positions and angles, and start a thread to monitor conditions.
# The class provides methods to read and write data to text files, convert coordinates to angles, calibrate the legs,
# set leg angles, check point validity, and transform coordinates.
class Control:
    def __init__(self):
        self.imu = IMU()    # Initialize the IMU class
        self.servo = Servo()    # Initialize the Servo class
        self.movement_flag = 0x01 
        self.relaxation_flag = False 
        self.pid_controller = Incremental_PID(0.500, 0.00, 0.0025) # Initialize the PID controller
        self.servo_power_disable = OutputDevice(4) # Initialize the servo power disable pin
        self.servo_power_disable.off()
        self.status_flag = 0x00
        self.timeout = 0
        self.body_height = -25
        self.body_points = [[137.1, 189.4, self.body_height], [225, 0, self.body_height], [137.1, -189.4, self.body_height], 
                           [-137.1, -189.4, self.body_height], [-225, 0, self.body_height], [-137.1, 189.4, self.body_height]]
        self.calibration_leg_positions = self.read_from_txt('point')
        self.leg_positions = [[140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0]]
        self.calibration_angles = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.current_angles = [[90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0]]
        self.command_queue = ['', '', '', '', '', '']
        self.calibrate()
        self.set_leg_angles()
        self.condition_thread = threading.Thread(target=self.condition_monitor)
        self.Thread_conditiona = threading.Condition()


# The read_from_txt method reads data from a text file and returns it as a list of lists.
# So it basically reads the data from a file and converts it into a format that can be used in the program.
    def read_from_txt(self, filename):
        with open(filename + ".txt", "r") as file:
            lines = file.readlines()
            data = [list(map(int, line.strip().split("\t"))) for line in lines]
        return data


# The save_to_txt method saves data to a text file. So it takes the data and writes it to a file in a specific format. It is used 
# to save the calibration leg positions to a file.
    def save_to_txt(self, data, filename):
        with open(filename + '.txt', 'w') as file:
            for row in data:
                file.write('\t'.join(map(str, row)) + '\n')

# The coordinate_to_angle method converts 3D coordinates to angles using inverse kinematics.
# So it basically takes the x, y, and z coordinates of a point and calculates the angles needed to reach that point.
# The angles are returned in degrees. It is used to calculate the angles for the legs based on their positions.
    def coordinate_to_angle(self, x, y, z, l1=33, l2=90, l3=110):
        a = math.pi / 2 - math.atan2(z, y)
        x_3 = 0
        x_4 = l1 * math.sin(a)
        x_5 = l1 * math.cos(a)
        l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)
        w = self.restrict_value((x - x_3) / l23, -1, 1)
        v = self.restrict_value((l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23), -1, 1)
        u = self.restrict_value((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2), -1, 1)
        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))
        return round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))
    

# The angle_to_coordinate method converts angles to 3D coordinates using forward kinematics. It is used for calculating the
# coordinates of the legs based on their angles. The angles are given in degrees, and the method returns the x, y, and z coordinates.
    def angle_to_coordinate(self, a, b, c, l1=33, l2=90, l3=110):
        a = math.pi / 180 * a
        b = math.pi / 180 * b
        c = math.pi / 180 * c
        x = round(l3 * math.sin(b + c) + l2 * math.sin(b))
        y = round(l3 * math.sin(a) * math.cos(b + c) + l2 * math.sin(a) * math.cos(b) + l1 * math.sin(a))
        z = round(l3 * math.cos(a) * math.cos(b + c) + l2 * math.cos(a) * math.cos(b) + l1 * math.cos(a))
        return x, y, z
    

# The calibrate method calibrates the leg positions and angles. It sets the initial positions and angles of the legs based on the
# calibration leg positions. It calculates the angles needed to reach the calibration positions and stores them in the calibration_angles list.
    def calibrate(self):
        self.leg_positions = [[140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0]]
        for i in range(6):
            self.calibration_angles[i][0], self.calibration_angles[i][1], self.calibration_angles[i][2] = self.coordinate_to_angle(
                -self.calibration_leg_positions[i][2], self.calibration_leg_positions[i][0], self.calibration_leg_positions[i][1])
        for i in range(6):
            self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = self.coordinate_to_angle(
                -self.leg_positions[i][2], self.leg_positions[i][0], self.leg_positions[i][1])
        for i in range(6):
            self.calibration_angles[i][0] = self.calibration_angles[i][0] - self.current_angles[i][0]
            self.calibration_angles[i][1] = self.calibration_angles[i][1] - self.current_angles[i][1]
            self.calibration_angles[i][2] = self.calibration_angles[i][2] - self.current_angles[i][2]


# The set_leg_angles method sets the angles of the legs based on their positions and the calibration angles. It checks if the
# coordinates are valid and then calculates the angles for each leg. The angles are set using the servo class.
    def set_leg_angles(self):
        if self.check_point_validity():
            for i in range(6):
                self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = self.coordinate_to_angle(
                    -self.leg_positions[i][2], self.leg_positions[i][0], self.leg_positions[i][1])
            for i in range(3):
                self.current_angles[i][0] = self.restrict_value(self.current_angles[i][0] + self.calibration_angles[i][0], 0, 180)
                self.current_angles[i][1] = self.restrict_value(90 - (self.current_angles[i][1] + self.calibration_angles[i][1]), 0, 180)
                self.current_angles[i][2] = self.restrict_value(self.current_angles[i][2] + self.calibration_angles[i][2], 0, 180)
                self.current_angles[i + 3][0] = self.restrict_value(self.current_angles[i + 3][0] + self.calibration_angles[i + 3][0], 0, 180)
                self.current_angles[i + 3][1] = self.restrict_value(90 + self.current_angles[i + 3][1] + self.calibration_angles[i + 3][1], 0, 180)
                self.current_angles[i + 3][2] = self.restrict_value(180 - (self.current_angles[i + 3][2] + self.calibration_angles[i + 3][2]), 0, 180)
            # Leg 1
            self.servo.set_servo_angle(15, self.current_angles[0][0])
            self.servo.set_servo_angle(14, self.current_angles[0][1])
            self.servo.set_servo_angle(13, self.current_angles[0][2])
            # Leg 2
            self.servo.set_servo_angle(12, self.current_angles[1][0])
            self.servo.set_servo_angle(11, self.current_angles[1][1])
            self.servo.set_servo_angle(10, self.current_angles[1][2])
            # Leg 3
            self.servo.set_servo_angle(9, self.current_angles[2][0])
            self.servo.set_servo_angle(8, self.current_angles[2][1])
            self.servo.set_servo_angle(31, self.current_angles[2][2])
            # Leg 6
            self.servo.set_servo_angle(16, self.current_angles[5][0])
            self.servo.set_servo_angle(17, self.current_angles[5][1])
            self.servo.set_servo_angle(18, self.current_angles[5][2])
            # Leg 5
            self.servo.set_servo_angle(19, self.current_angles[4][0])
            self.servo.set_servo_angle(20, self.current_angles[4][1])
            self.servo.set_servo_angle(21, self.current_angles[4][2])
            # Leg 4
            self.servo.set_servo_angle(22, self.current_angles[3][0])
            self.servo.set_servo_angle(23, self.current_angles[3][1])
            self.servo.set_servo_angle(27, self.current_angles[3][2])
        else:
            print("This coordinate point is out of the active range")

    def check_point_validity(self):
        is_valid = True
        leg_lengths = [0] * 6
        for i in range(6):
            leg_lengths[i] = math.sqrt(self.leg_positions[i][0] ** 2 + self.leg_positions[i][1] ** 2 + self.leg_positions[i][2] ** 2)
        for length in leg_lengths:
            if length > 248 or length < 90:
                is_valid = False
        return is_valid

    def condition_monitor(self):
        while True:
            if (time.time() - self.timeout) > 10 and self.timeout != 0 and self.command_queue[0] == '':
                self.timeout = time.time()
                self.relax(True)
                self.status_flag = 0x00
            if cmd.CMD_POSITION in self.command_queue and len(self.command_queue) == 4:
                if self.status_flag != 0x01:
                    self.relax(False)
                x = self.restrict_value(int(self.command_queue[1]), -40, 40)
                y = self.restrict_value(int(self.command_queue[2]), -40, 40)
                z = self.restrict_value(int(self.command_queue[3]), -20, 20)
                self.move_position(x, y, z)
                self.status_flag = 0x01
                self.command_queue = ['', '', '', '', '', '']
            elif cmd.CMD_ATTITUDE in self.command_queue and len(self.command_queue) == 4:
                if self.status_flag != 0x02:
                    self.relax(False)
                roll = self.restrict_value(int(self.command_queue[1]), -15, 15)
                pitch = self.restrict_value(int(self.command_queue[2]), -15, 15)
                yaw = self.restrict_value(int(self.command_queue[3]), -15, 15)
                points = self.calculate_posture_balance(roll, pitch, yaw)
                self.transform_coordinates(points)
                self.set_leg_angles()
                self.status_flag = 0x02
                self.command_queue = ['', '', '', '', '', '']
            elif cmd.CMD_MOVE in self.command_queue and len(self.command_queue) == 6:
                if self.command_queue[2] == "0" and self.command_queue[3] == "0":
                    self.run_gait(self.command_queue)
                    self.command_queue = ['', '', '', '', '', '']
                else:
                    if self.status_flag != 0x03:
                        self.relax(False)
                    self.run_gait(self.command_queue)
                    self.status_flag = 0x03
            elif cmd.CMD_BALANCE in self.command_queue and len(self.command_queue) == 2:
                if self.command_queue[1] == "1":
                    self.command_queue = ['', '', '', '', '', '']
                    if self.status_flag != 0x04:
                        self.relax(False)
                    self.status_flag = 0x04
                    self.imu6050()
            elif cmd.CMD_CALIBRATION in self.command_queue:
                self.timeout = 0
                self.calibrate()
                self.set_leg_angles()
                if len(self.command_queue) >= 2:
                    if self.command_queue[1] == "one":
                        self.calibration_leg_positions[0][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[0][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[0][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "two":
                        self.calibration_leg_positions[1][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[1][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[1][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "three":
                        self.calibration_leg_positions[2][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[2][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[2][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "four":
                        self.calibration_leg_positions[3][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[3][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[3][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "five":
                        self.calibration_leg_positions[4][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[4][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[4][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "six":
                        self.calibration_leg_positions[5][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[5][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[5][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "save":
                        self.save_to_txt(self.calibration_leg_positions, 'point')
                self.command_queue = ['', '', '', '', '', '']

    def relax(self, flag):
        if flag:
            self.servo.relax()
        else:
            self.set_leg_angles()

    def transform_coordinates(self, points):
        # Leg 1
        self.leg_positions[0][0] = points[0][0] * math.cos(54 / 180 * math.pi) + points[0][1] * math.sin(54 / 180 * math.pi) - 94
        self.leg_positions[0][1] = -points[0][0] * math.sin(54 / 180 * math.pi) + points[0][1] * math.cos(54 / 180 * math.pi)
        self.leg_positions[0][2] = points[0][2] - 14
        # Leg 2
        self.leg_positions[1][0] = points[1][0] * math.cos(0 / 180 * math.pi) + points[1][1] * math.sin(0 / 180 * math.pi) - 85
        self.leg_positions[1][1] = -points[1][0] * math.sin(0 / 180 * math.pi) + points[1][1] * math.cos(0 / 180 * math.pi)
        self.leg_positions[1][2] = points[1][2] - 14
        # Leg 3
        self.leg_positions[2][0] = points[2][0] * math.cos(-54 / 180 * math.pi) + points[2][1] * math.sin(-54 / 180 * math.pi) - 94
        self.leg_positions[2][1] = -points[2][0] * math.sin(-54 / 180 * math.pi) + points[2][1] * math.cos(-54 / 180 * math.pi)
        self.leg_positions[2][2] = points[2][2] - 14
        # Leg 4
        self.leg_positions[3][0] = points[3][0] * math.cos(-126 / 180 * math.pi) + points[3][1] * math.sin(-126 / 180 * math.pi) - 94
        self.leg_positions[3][1] = -points[3][0] * math.sin(-126 / 180 * math.pi) + points[3][1] * math.cos(-126 / 180 * math.pi)
        self.leg_positions[3][2] = points[3][2] - 14
        # Leg 5
        self.leg_positions[4][0] = points[4][0] * math.cos(180 / 180 * math.pi) + points[4][1] * math.sin(180 / 180 * math.pi) - 85
        self.leg_positions[4][1] = -points[4][0] * math.sin(180 / 180 * math.pi) + points[4][1] * math.cos(180 / 180 * math.pi)
        self.leg_positions[4][2] = points[4][2] - 14
        # Leg 6
        self.leg_positions[5][0] = points[5][0] * math.cos(126 / 180 * math.pi) + points[5][1] * math.sin(126 / 180 * math.pi) - 94
        self.leg_positions[5][1] = -points[5][0] * math.sin(126 / 180 * math.pi) + points[5][1] * math.cos(126 / 180 * math.pi)
        self.leg_positions[5][2] = points[5][2] - 14

    def restrict_value(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value

    def map_value(self, value, from_low, from_high, to_low, to_high):
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

# The move_position method moves the body to a new position based on the x, y, and z coordinates.
# It updates the body points and calculates the new angles for the legs. The method uses a deep copy of the body points to avoid modifying the original data.
    def move_position(self, x, y, z):
        points = copy.deepcopy(self.body_points)
        for i in range(6):
            points[i][0] = self.body_points[i][0] - x
            points[i][1] = self.body_points[i][1] - y
            points[i][2] = -30 - z
            self.body_height = points[i][2]
            self.body_points[i][2] = points[i][2]
        self.transform_coordinates(points)
        self.set_leg_angles()

# The calculate_posture_balance method calculates the foot positions based on the roll, pitch, and yaw angles.
# It uses rotation matrices to transform the coordinates of the foot points and returns the new positions.
    def calculate_posture_balance(self, roll, pitch, yaw):
        position = np.mat([0.0, 0.0, self.body_height]).T
        rpy = np.array([roll, pitch, yaw]) * math.pi / 180
        roll_angle, pitch_angle, yaw_angle = rpy[0], rpy[1], rpy[2]
        rotation_x = np.mat([[1, 0, 0],
                             [0, math.cos(pitch_angle), -math.sin(pitch_angle)],
                             [0, math.sin(pitch_angle), math.cos(pitch_angle)]])
        rotation_y = np.mat([[math.cos(roll_angle), 0, -math.sin(roll_angle)],
                             [0, 1, 0],
                             [math.sin(roll_angle), 0, math.cos(roll_angle)]])
        rotation_z = np.mat([[math.cos(yaw_angle), -math.sin(yaw_angle), 0],
                             [math.sin(yaw_angle), math.cos(yaw_angle), 0],
                             [0, 0, 1]])
        rotation_matrix = rotation_x * rotation_y * rotation_z
        body_structure = np.mat([[55, 76, 0],
                                [85, 0, 0],
                                [55, -76, 0],
                                [-55, -76, 0],
                                [-85, 0, 0],
                                [-55, 76, 0]]).T
        footpoint_structure = np.mat([[137.1, 189.4, 0],
                                     [225, 0, 0],
                                     [137.1, -189.4, 0],
                                     [-137.1, -189.4, 0],
                                     [-225, 0, 0],
                                     [-137.1, 189.4, 0]]).T
        ab = np.mat(np.zeros((3, 6)))
        foot_positions = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in range(6):
            ab[:, i] = position + rotation_matrix * footpoint_structure[:, i]
            foot_positions[i][0] = ab[0, i]
            foot_positions[i][1] = ab[1, i]
            foot_positions[i][2] = ab[2, i]
        return foot_positions

# The imu6050 method is used to read data from the IMU sensor and adjust the leg angles accordingly.
# It uses a PID controller to calculate the roll and pitch angles and then transforms the coordinates of the legs.
    def imu6050(self):
        old_roll = 0
        old_pitch = 0
        points = self.calculate_posture_balance(0, 0, 0)
        self.transform_coordinates(points)
        self.set_leg_angles()
        time.sleep(2)
        self.imu.Error_value_accel_data, self.imu.Error_value_gyro_data = self.imu.calculate_average_sensor_data()
        time.sleep(1)
        while True:
            if self.command_queue[0] != "":
                break
            time.sleep(0.02)
            roll, pitch, yaw = self.imu.update_imu_state()
            roll = self.pid_controller.pid_calculate(roll)
            pitch = self.pid_controller.pid_calculate(pitch)
            points = self.calculate_posture_balance(roll, pitch, 0)
            self.transform_coordinates(points)
            self.set_leg_angles()

# The run_gait method executes the gait based on the provided data. It takes the gait type, x and y coordinates, speed, and rotation angle.
# It calculates the new positions of the legs based on the gait type and updates the leg angles accordingly.
# The gait type can be "1" for tripod or "2" for ripple. The method uses trigonometric functions to calculate the new positions.

# Gait 1 = Tripod gait: The legs move in a triangular pattern, with three legs moving at a time.
# Gait 2 = Ripple gait: The legs move in a wave-like pattern, with all legs moving in a coordinated manner.

    def run_gait(self, data, Z=40, F=64):  # Example: data=['CMD_MOVE', '1', '0', '25', '10', '0']
        gait = data[1]
        x = self.restrict_value(int(data[2]), -35, 35)
        y = self.restrict_value(int(data[3]), -35, 35)
        if gait == "1":
            F = round(self.map_value(int(data[4]), 2, 10, 126, 22))
        else:
            F = round(self.map_value(int(data[4]), 2, 10, 171, 45))
        angle = int(data[5])
        z = Z / F
        delay = 0.01
        points = copy.deepcopy(self.body_points)
        xy = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        for i in range(6):
            xy[i][0] = ((points[i][0] * math.cos(angle / 180 * math.pi) + points[i][1] * math.sin(angle / 180 * math.pi) - points[i][0]) + x) / F
            xy[i][1] = ((-points[i][0] * math.sin(angle / 180 * math.pi) + points[i][1] * math.cos(angle / 180 * math.pi) - points[i][1]) + y) / F
        if x == 0 and y == 0 and angle == 0:
            self.transform_coordinates(points)
            self.set_leg_angles()
        elif gait == "1":
            for j in range(F):
                for i in range(3):
                    if j < (F / 8):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] + 8 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] + 8 * xy[2 * i + 1][1]
                        points[2 * i + 1][2] = Z + self.body_height
                    elif j < (F / 4):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][2] = points[2 * i + 1][2] - z * 8
                    elif j < (3 * F / 8):
                        points[2 * i][2] = points[2 * i][2] + z * 8
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (5 * F / 8):
                        points[2 * i][0] = points[2 * i][0] + 8 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] + 8 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (3 * F / 4):
                        points[2 * i][2] = points[2 * i][2] - z * 8
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (7 * F / 8):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][2] = points[2 * i + 1][2] + z * 8
                    elif j < (F):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] + 8 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] + 8 * xy[2 * i + 1][1]
                self.transform_coordinates(points)
                self.set_leg_angles()
                time.sleep(delay)
        elif gait == "2":
            number = [5, 2, 1, 0, 3, 4]
            for i in range(6):
                for j in range(int(F / 6)):
                    for k in range(6):
                        if number[i] == k:
                            if j < int(F / 18):
                                points[k][2] += 18 * z
                            elif j < int(F / 9):
                                points[k][0] += 30 * xy[k][0]
                                points[k][1] += 30 * xy[k][1]
                            elif j < int(F / 6):
                                points[k][2] -= 18 * z
                        else:
                            points[k][0] -= 2 * xy[k][0]
                            points[k][1] -= 2 * xy[k][1]
                    self.transform_coordinates(points)
                    self.set_leg_angles()
                    time.sleep(delay)

if __name__ == '__main__':
    pass