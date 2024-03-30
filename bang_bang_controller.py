#!/usr/bin/env python3
import numpy as np

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import time

def shift(a, new_val):
    new_a = a.copy()
    new_a[:-1] = a[1:]
    new_a[-1] = new_val

class RobotController:
    def __init__(self):
        self.robot_name = "locobot"
        
        full_rd_name = "/" + self.robot_name + "/robot_description"
        self.locobot_urdf = URDF.from_parameter_server(key=full_rd_name)
        self.locobot_tree = kdl_tree_from_urdf_model(self.locobot_urdf)
        
        self.locobot = InterbotixLocobotXS("locobot_wx250s", "mobile_wx250s") # set up the arm
        self.locobot.arm.go_to_home_pose()
        
        self.window_size = 3
        self.waist_threshold = 150
        self.shoulder_threshold = 70
        self.base_rot_factor = 1.5
        self.base_vel_factor = 0.25

    def get_joint_effort(self):
        return np.array([self.locobot.arm.core.joint_states.effort[self.locobot.arm.core.js_index_map[name]] for name in self.locobot.arm.group_info.joint_names])

    def run(self):
        w = self.window_size
        waist_sum_window = np.array([0] * w)
        shoulder_sum_window = np.array([0] * w)
        for i in range(w):
            e = self.get_joint_effort()
            shoulder_sum_window[i] = e[1]
            waist_sum_window[i] = e[0]
            time.sleep(0.1)
        
        while True:
            # rotation
            waist_state = np.average(waist_sum_window)
            waist = self.get_joint_effort()[0]
            waist_diff = waist - waist_state
            wdirection = 1 if waist_diff < -self.waist_threshold else -1 if waist_diff > self.waist_threshold else 0
            
            # forward/backward motion
            shoulder_state = np.average(shoulder_sum_window)
            shoulder = self.get_joint_effort()[1]
            shoulder_diff = shoulder - shoulder_state
            sdirection = 1 if shoulder_diff < -self.shoulder_threshold else -1 if shoulder_diff > self.shoulder_threshold else 0
            
            shift(shoulder_sum_window, shoulder)
            shift(waist_sum_window, waist)

            if sdirection != 0 or wdirection != 0:
                self.locobot.base.move(yaw=wdirection * self.base_rot_factor, x=sdirection * self.base_vel_factor, duration=0.1)
            else:
                time.sleep(0.1)

controller = RobotController()
controller.run()
