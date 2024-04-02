#!/usr/bin/env python3
import numpy as np

from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Twist, Vector3
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from interbotix_xs_msgs.msg import *
import time

import signal
import sys
import rospy

class RobotController:
    def __init__(self):
        self.robot_name = "locobot"
        
        self.locobot_rate = 120
        self.control_rate = 100
        
        # for BB controller
        self.base_rot_factor = 1.5
        self.base_vel_factor = 0.25
        
        # band limits for torque sensors
        self.shoulder_threshold = 80
        self.elbow_threshold = 380
        self.waist_threshold = 220
        
        # p, pd, pid controller paramerters
        self.rot_kp = -2e-2
        self.lin_kp = -1e-3
        self.rot_kd = -1e-4
        self.lin_kd = -1e-4
        self.rot_ki = -2e-4
        self.lin_ki = -3e-5
        self.control = self.control_pd

        self.tperiod = 1 / self.control_rate
        rospy.set_param(f"/{self.robot_name}/joint_state_publisher/rate", self.locobot_rate)
        
        self.locobot_urdf = URDF.from_parameter_server(key=f"/{self.robot_name}/robot_description")
        self.locobot_tree = kdl_tree_from_urdf_model(self.locobot_urdf)
        
        self.locobot = InterbotixLocobotXS("locobot_wx250s", "mobile_wx250s")

    def get_joint_effort(self):
        return np.array([self.locobot.arm.core.joint_states.effort[self.locobot.arm.core.js_index_map[name]] for name in self.locobot.arm.group_info.joint_names])

    def get_joint_position(self):
        return np.array([self.locobot.arm.core.joint_states.position[self.locobot.arm.core.js_index_map[name]] for name in self.locobot.arm.group_info.joint_names])

    def run(self):
        self.init_controller()
        print("Started the controller...")
        
        waist_vals = np.array([0] * 3, dtype=np.float64)
        shoulder_vals = np.array([0] * 3, dtype=np.float64)
        elbow_vals = np.array([0] * 3, dtype=np.float64)
        for i in range(3):
            t = self.get_joint_effort()
            waist_vals[i] = t[0]
            shoulder_vals[i] = t[1]
            elbow_vals[i] = t[2]
            time.sleep(self.tperiod)
        pos = 0

        while True:
            rot, linear = self.control()
            
            # recalibrate
            if linear == 0:
                t = self.get_joint_effort()
                waist_vals[i] = t[0]
                shoulder_vals[i] = t[1]
                elbow_vals[i] = t[2]
                pos = (pos + 1) % 3
                self.shoulder_baseline = np.average(shoulder_vals)
                self.waist_baseline = np.average(waist_vals)
                self.elbow_baseline = np.average(elbow_vals)

            self.locobot.base.pub_base_command.publish(Twist(linear=Vector3(x=linear), angular=Vector3(z=rot)))
            time.sleep(self.tperiod)

    def init_controller(self):
        self.locobot.arm.go_to_home_pose()
        self.prev_rot_error = 0
        self.prev_lin_error = 0
        self.int_rot = 0
        self.int_lin = 0
        self.prev_time = time.time()
        
        waist = sholder = elbow = 0
        for _ in range(100):
            waist += self.get_joint_effort()[0]
            sholder += self.get_joint_effort()[1]
            elbow += self.get_joint_effort()[2]
            time.sleep(self.tperiod)
        waist /= 100
        sholder /= 100
        elbow /= 100
        
        self.shoulder_baseline = sholder
        self.elbow_baseline = elbow
        self.waist_baseline = waist
    
    def get_linear_actuations(self):
        # forward/backward motion
        upper_shoulder, lower_shoulder = self.shoulder_baseline + self.shoulder_threshold, self.shoulder_baseline - self.shoulder_threshold
        upper_elbow, lower_elbow = self.elbow_baseline + self.elbow_threshold, self.elbow_baseline - self.elbow_threshold
        
        middle_shoulder = lower_shoulder + (upper_shoulder - lower_shoulder) / 2
        middle_elbow = lower_elbow + (upper_elbow - lower_elbow) / 2   
        
        shoulder = self.get_joint_effort()[1]
        elbow = self.get_joint_effort()[2]
        if shoulder > lower_shoulder and shoulder < upper_shoulder:
            shoulder = middle_shoulder
        if elbow > lower_elbow and elbow < upper_elbow:
            elbow = middle_elbow
        shoulder -= middle_shoulder
        elbow -= middle_elbow
        
        combined = shoulder - 2*elbow
        
        return combined
    
    def get_rotation_actuation(self):
        # rotation
        upper, lower = self.waist_baseline + self.waist_threshold, self.waist_baseline - self.waist_threshold
        middle = lower + (upper - lower) / 2
        
        waist = self.get_joint_effort()[0]
        if waist > lower and waist < upper:
            waist = middle
        waist -= middle
        return waist

    def control_bb(self):
        rot = self.get_rotation_actuation()
        linear = self.get_linear_actuations()

        wdirection = 1 if rot < 0 else -1 if rot > 0 else 0
        sdirection = 1 if linear < 0 else -1 if linear > 0 else 0

        return wdirection * self.base_rot_factor, sdirection * self.base_vel_factor

    def control_p(self):
        rot = self.get_rotation_actuation()
        lin = self.get_linear_actuations()
        
        wdirection = self.rot_kp * rot
        sdirection = self.lin_kp * lin
        
        return wdirection, sdirection
    
    def control_pd(self):
        rot = self.get_rotation_actuation()
        lin = self.get_linear_actuations()
        
        derivative_rot = (rot - self.prev_rot_error) / (time.time() - self.prev_time)
        derivative_lin = (lin - self.prev_lin_error) / (time.time() - self.prev_time)

        self.prev_rot_error = rot
        self.prev_lin_error = lin
        self.prev_time = time.time()

        wdirection = self.rot_kp * rot + self.rot_kd * derivative_rot
        sdirection = self.lin_kp * lin + self.lin_kd * derivative_lin

        return wdirection, sdirection

    def control_pid(self):
        rot = self.get_rotation_actuation()
        lin = self.get_linear_actuations()
        
        delta = (time.time() - self.prev_time)
        derivative_rot = (rot - self.prev_rot_error) / delta
        derivative_lin = (lin - self.prev_lin_error) / delta
        self.int_rot += rot * delta
        self.int_lin += lin * delta

        self.prev_rot_error = rot
        self.prev_lin_error = lin
        self.prev_time = time.time()

        wdirection = self.rot_kp * rot + self.rot_kd * derivative_rot + self.rot_ki * self.int_rot
        sdirection = self.lin_kp * lin + self.lin_kd * derivative_lin + self.lin_ki * self.int_lin

        return wdirection, sdirection

def handler(signum, frame):
    print("robot exiting")
    sys.exit(0)

signal.signal(signal.SIGTSTP, handler)
controller = RobotController()
controller.run()
