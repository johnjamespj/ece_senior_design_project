#!/usr/bin/env python3
import numpy as np

from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Twist, Vector3
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from interbotix_xs_msgs.msg import *
import PyKDL as kdl
import time

import signal
import sys
import rospy

w250 = (0.0009, -0.12)
w350 = (0.00175, -0.175)

sleep_pos = np.array([  [ 0.5839743,   0.12362206,  0.80230393,  0.11089897],
                        [-0.06760064,  0.99230926, -0.10369423, -0.01209753],
                        [-0.80895251,  0.00631851,  0.58784004,  0.01097851],
                        [ 0.,          0.,          0.,          1.,        ]]  )

f = open("output/c1.npy", "wb")

class RobotController:
    def __init__(self):
        self.robot_name = "locobot"
        self.urdf = URDF.from_xml_string(rospy.get_param(f"/{self.robot_name}/robot_description"))
        self.kdl_tree = kdl_tree_from_urdf_model(self.urdf)
        self.arm_chain = self.kdl_tree.getChain(f"{self.robot_name}/arm_base_link", f"{self.robot_name}/ee_gripper_link")

        self.locobot_rate = 120
        self.control_rate = 100

        # for BB controller
        self.base_rot_factor = 1.5
        self.base_vel_factor = 0.4

        # band limits for torque sensors
        self.shoulder_upper = -200
        self.elbow_upper = -160
        self.waist_upper = 200
        self.waist_lower = -160

        # p, pd, pid controller paramerters
        self.rot_kp = -5e-2
        self.lin_kp = -10e-3
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

    def get_joint_torques(self):
        efforts = self.get_joint_effort()
        torques = np.zeros(6)
        torques[:-1] = w350[0] * efforts[:-1] + w350[1]
        torques[-1] = w250[0] * efforts[-1] + w250[1]
        return torques

    def get_jacobian(self):
        end_effector_frame = kdl.Frame()
        
        # Create a solver for the chain
        solver = kdl.ChainJntToJacSolver(self.arm_chain)

        # Define the joint positions (angles)
        joint_positions = kdl.JntArray(self.arm_chain.getNrOfJoints())
        
        positions = self.get_joint_position()
        for i in range(len(positions)):
            joint_positions[i] = positions[i]
        
        jacobian = kdl.Jacobian(self.arm_chain.getNrOfJoints())
        solver.JntToJac(joint_positions, jacobian)
        
        jacobian_np = np.zeros((jacobian.rows(), jacobian.columns()))
        for i in range(jacobian.rows()):
            for j in range(jacobian.columns()):
                jacobian_np[i, j] = jacobian[i, j]
        
        torques = self.get_joint_torques()
        print(jacobian_np)
        print(torques)
        print(np.matmul(np.linalg.inv(jacobian_np.T), torques))
        return jacobian_np

    def run(self):
        self.init_controller()
        print("Started the controller...")

        while True:
            np.save(f, time.time())
            rot, linear = self.control()
            self.locobot.base.pub_base_command.publish(Twist(linear=Vector3(x=linear), angular=Vector3(z=rot)))
            np.save(f, np.array([rot, linear]))
            time.sleep(self.tperiod)

    def init_controller(self):
        positions = [0] * self.locobot.arm.group_info.num_joints
        positions[5] = np.radians(90) # forarm roll
        self.locobot.arm.publish_positions(positions)
        self.locobot.gripper.set_pressure(1)
        self.locobot.gripper.open(0.4)
        time.sleep(1)
        self.locobot.gripper.close(1)

        
        self.prev_rot_error = 0
        self.prev_lin_error = 0
        self.int_rot = 0
        self.int_lin = 0
        self.prev_time = time.time()
    
    def get_linear_actuations(self):
        efforts = self.get_joint_effort()

        forwardEffort = max(efforts[2], self.elbow_upper) - self.elbow_upper
        backwardEffort = max(efforts[1], self.shoulder_upper) - self.shoulder_upper

        lin = backwardEffort - 2*forwardEffort
        np.save(f, lin)
        return lin
    
    def get_rotation_actuation(self):
        # rotation
        upper, lower = self.waist_upper, self.waist_lower
        middle = lower + (upper - lower) / 2
        waist = self.get_joint_effort()[0]
        if waist > lower and waist < upper:
            waist = middle

        waist -= middle
        np.save(f, waist)
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

    def stop(self):
        self.locobot.gripper.set_pressure(1)
        self.locobot.base.pub_base_command.publish(Twist(linear=Vector3(x=0), angular=Vector3(z=0)))
        self.locobot.gripper.open(1)
        self.locobot.arm.go_to_sleep_pose()
        self.locobot.gripper.close(1)

controller = RobotController()
def handler(signum, frame):
    controller.stop()
    print("robot exiting")
    sys.exit(0)

signal.signal(signal.SIGTSTP, handler)
controller.run()
