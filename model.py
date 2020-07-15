# -*- coding: utf-8 -*-

import geometry_math
import time
from math import pi, radians, degrees
from comunicate import COMM


class robot_model(object):
    def __init__(self, l1=80, l2=80, l3=72, pin1=8, pin2=9, pin3=10, pin4=11,
                 serial_port='COM4', baud_rate=9600,
                 offset_base=1, joint_min_base=-pi/4, joint_max_base=pi/4,
                 offset_lower_arm=-14, joint_min_lower_arm=pi/4, joint_max_lower_arm=3*pi,
                 offset_upper_arm=25, joint_min_upper_arm=-pi/4, joint_max_upper_arm=pi/4,
                 offset_gripper=0, joint_min_gripper=0, joint_max_gripper=pi/4):
        self.armLink = [l1, l2, l3]
        # self.servoPin = {'base': pin1, 'rArm': pin2, 'fArm': pin3, 'gripper': pin4}
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.joint_info = {'base': {'mark': 'b', 'offset': offset_base, 'min_joint_angle': joint_min_base, 'max_joint_angle': joint_max_base},
                           'lower_arm': {'mark': 'l', 'offset': offset_lower_arm, 'min_joint_angle': joint_min_lower_arm, 'max_joint_angle': joint_max_lower_arm},
                           'upper_arm': {'mark': 'u', 'offset': offset_upper_arm, 'min_joint_angle': joint_min_upper_arm, 'max_joint_angle': joint_max_upper_arm},
                           'gripper': {'mark': 'g', 'offset': offset_gripper, 'min_joint_angle': joint_min_gripper, 'max_joint_angle': joint_max_gripper}}
        self.connect_serial = None
        self.cur_cart_pos = [0, 0, 0, 0]

    def connect(self):
        self.connect_serial = COMM(self.serial_port, self.baud_rate)

    def set_servo(self, servo_angles, reverse=False):
        servo_angles = list(map(lambda x, y: self.calibrate_servo(x, y), servo_angles, [0, 1, 2]))

        if self.connect_serial is not None:
            if reverse:
                joint_info = list(self.joint_info.keys())[:3][::-1]
                servo_angles = servo_angles[:3][::-1]
            else:
                joint_info = list(self.joint_info.keys())[:3]
            for i, arm_name in enumerate(joint_info):
                command = str(self.joint_info[arm_name]['mark']) + str(int(servo_angles[i])) + ' '
                print(command)
                self.connect_serial.ser.write(command.encode())
            # self.connected_serial.ser.write(str(self.servoPin[arm]) + str(int(angle)))
        else:
            print('Can not connect controller!')

    def calibrate_servo(self, servo_angle, arm_type):
        # 0 - base; 1 - lower arm; 2 - upper arm; 3 - gripper
        tmp_dict = {0: 'base', 1: 'lower_arm', 2: 'upper_arm', 3: 'gripper'}
        offset = self.joint_info[tmp_dict[arm_type]]['offset']
        calibrated_servo_angle = servo_angle + offset

        return calibrated_servo_angle

    def forward_ik(self, servo_angles):
        # servo_angles = list(map(lambda x, y: self.calibrate_servo(x, y), servo_angles, [0, 1, 2]))
        joint_angles = [pi / 2 - radians(servo_angles[0]), pi - radians(servo_angles[1]), - pi/2 + radians(servo_angles[2])]

        # Calculate u,v coordinates for arm
        u01, v01 = geometry_math.polar2cart(self.armLink[0], joint_angles[1])
        u12, v12 = geometry_math.polar2cart(self.armLink[1], joint_angles[2])

        # Add vectors
        u = u01 + u12 + self.armLink[2]
        v = v01 + v12

        # Calculate in 3D space - note x/y reversal!
        y, x = geometry_math.polar2cart(u, joint_angles[0])
        z = v
        return x, y, z

    def inverse_ik(self, cart_pos):
        # Solve top-down view
        r, th0 = geometry_math.cart2polar(cart_pos[1], cart_pos[0])
        r -= self.armLink[2]  # Account for the wrist length

        # In arm plane, convert to polar
        r_arm_plane, ang_arm_plane = geometry_math.cart2polar(r, cart_pos[2])

        parm_b = [0]
        parm_c = [0]
        # Solve arm inner angles as required
        if not geometry_math.cos_angle(self.armLink[1], self.armLink[0], r_arm_plane, parm_b) or not geometry_math.cos_angle(r_arm_plane, self.armLink[0], self.armLink[1], parm_c):
            servo_angles = []
        else:
            b = parm_b[0]
            c = parm_c[0]

            # Solve for servo angles from horizontal
            a0 = th0
            a1 = ang_arm_plane + b
            a2 = c + a1 - pi
            joint_angles = [a0, a1, a2]

            servo_angles = [pi/2 - joint_angles[0], pi - joint_angles[1], pi/2 + joint_angles[2]]
            servo_angles = list(map(lambda x: degrees(x), servo_angles))
            # servo_angles = list(map(lambda x, y: self.calibrate(x, y), servo_angles, [0, 1, 2]))

        return servo_angles

    def go_directly(self, cart_pos):
        angles = self.inverse_ik(cart_pos)
        angles.append(0)
        if len(angles) == 4:
            self.set_servo(angles)

            self.cur_cart_pos = cart_pos
            print("Go to: ", cart_pos)

    def goto_point(self, cart_pos):
        x0 = self.cur_cart_pos[0]
        y0 = self.cur_cart_pos[1]
        z0 = self.cur_cart_pos[2]
        x = cart_pos[0]
        y = cart_pos[1]
        z = cart_pos[2]

        dist = geometry_math.distance(x0, y0, z0, x, y, z)
        step = 10
        i = 0
        while i < dist:
            inter_car_pos = [x0 + (x - x0) * i / dist,
                             y0 + (y - y0) * i / dist,
                             z0 + (x - z0) * i / dist]
            self.go_directly(inter_car_pos)
            time.sleep(0.05)
            i += step

        self.go_directly(cart_pos)
        time.sleep(0.05)

    def open_gripper(self):
        command = 'g10'
        print(command)
        self.connect_serial.ser.write(command.encode())
        time.sleep(0.05)

    def close_gripper(self):
        command = 'g0'
        print(command)
        self.connect_serial.ser.write(command.encode())
        time.sleep(0.05)

    def reachable(self, cart_pos):
        angles = self.inverse_ik(cart_pos)
        reachable_state = True if len(angles) == 3 else False
        return reachable_state

    def get_pos(self):
        return self.cur_cart_pos
