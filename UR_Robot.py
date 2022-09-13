#coding=utf8
import time
import os
import random
import threading
import argparse
import matplotlib.pyplot as plt
import scipy as sc
from collections import namedtuple
# import utils
import socket
import select
import struct
import numpy as np
import math
from real.robotiq_gripper import RobotiqGripper
from real.realsenseD435 import RealsenseD435

class UR_Robot:
    def __init__(self, tcp_host_ip="192.168.50.100", tcp_port=30003, workspace_limits=None,
                 is_use_robotiq85=True, is_use_camera=True,):
        # Init varibles
        if workspace_limits is None:
            workspace_limits = [[0.3, 0.748], [-0.224, 0.224], [-0.255, -0.1]]
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        self.is_use_robotiq85 = is_use_robotiq85
        self.is_use_camera = is_use_camera
        self.workspace_limits=workspace_limits
        # self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


        # UR5 robot configuration
        # Default joint/tool speed configuration
        self.joint_acc = 1.4  # Safe: 1.4   8
        self.joint_vel = 1.05  # Safe: 1.05  3

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.01

        # Default tool speed configuration
        self.tool_acc = 1.2  # Safe: 0.5
        self.tool_vel = 0.25  # Safe: 0.2

        # Tool pose tolerance for blocking calls
        self.tool_pose_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

        # robotiq85 gripper configuration
        # if(self.is_use_robotiq85):
        #     # reference https://gitlab.com/sdurobotics/ur_rtde
        #     # Gripper activate
        #     self.gripper = RobotiqGripper()
        #     self.gripper.connect(self.tcp_host_ip, 63352)  # don't change the 63352 port
        #     self.gripper._reset()
        #     print("Activating gripper...")
        #     self.gripper.activate()
        #     time.sleep(1.5)
        
        # realsense configuration
        if(self.is_use_camera):
            # Fetch RGB-D data from RealSense camera
            self.camera = RealsenseD435()
            self.cam_intrinsics = self.camera.intrinsics  # get camera intrinsics
            # # Load camera pose (from running calibrate.py), intrinsics and depth scale
            # self.cam_pose = np.loadtxt('real/camera_pose.txt', delimiter=' ')
            # self.cam_depth_scale = np.loadtxt('real/camera_depth_scale.txt', delimiter=' ')
   

        # Default robot home joint configuration (the robot is up to air)
        self.home_joint_config = [-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             -(0 / 360.0) * 2 * np.pi, 0.0]

        # test
        self.get_camera_data()
        # self.testRobot()
    # Test for robot control
    def testRobot(self):
        try:
            print("Test for robot...")
            # self.move_j([-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  -(0 / 360.0) * 2 * np.pi, 0.0])
            # self.move_j([(57.04 / 360.0) * 2 * np.pi, (-65.26/ 360.0) * 2 * np.pi,
            #                  (73.52/ 360.0) * 2 * np.pi, (-100.89/ 360.0) * 2 * np.pi,
            #                  (-86.93/ 360.0) * 2 * np.pi, (-0.29/360)*2*np.pi])
            print("11")
            # self.open_gripper()
            # self.move_j([(57.03 / 360.0) * 2 * np.pi, (-56.67 / 360.0) * 2 * np.pi,
            #                   (88.72 / 360.0) * 2 * np.pi, (-124.68 / 360.0) * 2 * np.pi,
            #                   (-86.96/ 360.0) * 2 * np.pi, (-0.3/ 360) * 2 * np.pi])
            # self.close_gripper()
            # self.move_j([(57.04 / 360.0) * 2 * np.pi, (-65.26 / 360.0) * 2 * np.pi,
            #                   (73.52 / 360.0) * 2 * np.pi, (-100.89 / 360.0) * 2 * np.pi,
            #                   (-86.93 / 360.0) * 2 * np.pi, (-0.29 / 360) * 2 * np.pi])
            #
            #
            # self.move_j([-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  -(0 / 360.0) * 2 * np.pi, 0.0])
            self.move_j_p([-0.266,-0.618,0.541,0.872,2.976,0])
            self.move_p([-0.266, -0.618, 0.441, 0.872, 2.976, 0])
            # self.move_l([-0.266,-0.618,0.441,0.872,2.976,0.089])
            print("22")
            # move_c bug
            # self.move_c([-0.366, -0.518, 0.441, 0.872, 2.976, 0],[-0.266, -0.418, 0.441, 0.872, 2.976, 0])
        except:
            print("Test fail! Please check the ip address or integrity of the file")
    
    # joint control
    '''
    input:joint_configuration = joint angle
    '''
    def move_j(self, joint_configuration,k_acc=1,k_vel=1,t=0,r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # command: movej([joint_configuration],a,v,t,r)\n
        tcp_command = "movej([%f" % joint_configuration[0]  #"movej([]),a=,v=,\n"
        for joint_idx in range(1,6):
            tcp_command = tcp_command + (",%f" % joint_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (k_acc*self.joint_acc, k_vel*self.joint_vel)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
        while not all([np.abs(actual_joint_positions[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
            time.sleep(0.01)
        self.tcp_socket.close()
    # joint control
    '''
    input:tool_configuration=[position,angle]
    '''
    def move_j_p(self, tool_configuration,k_acc=1,k_vel=1,t=0,r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # command: movej([joint_configuration],a,v,t,r)\n
        tcp_command = "movej(get_inverse_kin(p[%f" % tool_configuration[0]  # "movej([]),a=,v=,\n"
        for joint_idx in range(1, 6):
            tcp_command = tcp_command + (",%f" % tool_configuration[joint_idx])
        tcp_command = tcp_command + "]),a=%f,v=%f)\n" % (k_acc * self.joint_acc, k_vel * self.joint_vel)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in
                       range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        self.tcp_socket.close()
    def move_p(self, tool_configuration,k_acc=1,k_vel=1,r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # command: movej([joint_configuration],a,v,t,r)\n
        tcp_command = "movep(p[%f" % tool_configuration[0]  # "movej([]),a=,v=,\n"
        for joint_idx in range(1, 6):
            tcp_command = tcp_command + (",%f" % tool_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (k_acc * self.tool_acc, k_vel * self.tool_vel)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in
                       range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        self.tcp_socket.close()
    
    # tool TCP control (Linear travel)
    def move_l(self, tool_configuration,k_acc=1,k_vel=1,t=0,r=0):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # command: movel([tool_configuration],a,v,t,r)\n
        tcp_command = "movel(p[%f" % tool_configuration[0]  #"movel([]),a=,v=,\n"
        for joint_idx in range(1,6):
            tcp_command = tcp_command + (",%f" % tool_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (k_acc*self.tool_acc, k_vel*self.tool_vel)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        self.tcp_socket.close()


    # �ӵ�ǰ��̬ͨ��pose_via��tool_configuration
    def move_c(self,pose_via,tool_configuration,k_acc=1,k_vel=1,r=0):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # command: movec([pose_via,tool_configuration],a,v,t,r)\n
        tcp_command = f"movec([{pose_via[0]},{pose_via[1]},{pose_via[2]},{pose_via[3]},{pose_via[4]},{pose_via[5]}], \
        [{tool_configuration[0]},{tool_configuration[1]},{tool_configuration[2]},{tool_configuration[3]},{tool_configuration[4]},{tool_configuration[5]}], \
        a={k_acc*self.tool_acc},v={k_vel*self.tool_vel})\n"

        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        self.tcp_socket.close()

    def go_home(self):
        self.move_j(self.home_joint_config)

    def restartReal(self):
        pass


    # get robot current state and information
    def get_state(self):
        self.tcp_cket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        state_data = self.tcp_socket.recv(1500)
        self.tcp_socket.close()
        return state_data
    
    # get robot current joint angles and cartesian pose
    def parse_tcp_state_data(self, data, subpasckage):
        dic = {'MessageSize': 'i', 'Time': 'd', 'q target': '6d', 'qd target': '6d', 'qdd target': '6d',
               'I target': '6d',
               'M target': '6d', 'q actual': '6d', 'qd actual': '6d', 'I actual': '6d', 'I control': '6d',
               'Tool vector actual': '6d', 'TCP speed actual': '6d', 'TCP force': '6d', 'Tool vector target': '6d',
               'TCP speed target': '6d', 'Digital input bits': 'd', 'Motor temperatures': '6d', 'Controller Timer': 'd',
               'Test value': 'd', 'Robot Mode': 'd', 'Joint Modes': '6d', 'Safety Mode': 'd', 'empty1': '6d',
               'Tool Accelerometer values': '3d',
               'empty2': '6d', 'Speed scaling': 'd', 'Linear momentum norm': 'd', 'SoftwareOnly': 'd',
               'softwareOnly2': 'd',
               'V main': 'd',
               'V robot': 'd', 'I robot': 'd', 'V actual': '6d', 'Digital outputs': 'd', 'Program state': 'd',
               'Elbow position': 'd', 'Elbow velocity': '3d'}
        ii = range(len(dic))
        for key, i in zip(dic, ii):
            fmtsize = struct.calcsize(dic[key])
            data1, data = data[0:fmtsize], data[fmtsize:]
            fmt = "!" + dic[key]
            dic[key] = dic[key], struct.unpack(fmt, data1)

        if subpasckage == 'joint_data':  # get joint data
            q_actual_tuple = dic["q actual"]
            joint_data= np.array(q_actual_tuple[1])
            return joint_data
        elif subpasckage == 'cartesian_info':
            Tool_vector_actual = dic["Tool vector actual"]  # get x y z rx ry rz
            cartesian_info = np.array(Tool_vector_actual[1])
            return cartesian_info

    ## robotiq85 gripper
    # get gripper position [0-255]  open:0 ,close:255
    def get_current_tool_pos(self):
        return self.gripper.get_current_position()       

    def log_gripper_info(self,gripper):
        print(f"Pos: {str(gripper.get_current_position())}  "
              f"Open: {gripper.is_open()} "
              f"Closed: {gripper.is_closed()} ")

    def close_gripper(self,speed=255,force=255):
        # position: int[0-255], speed: int[0-255], force: int[0-255]
        self.gripper.move_and_wait_for_pos(255, speed, force)
        self.log_gripper_info(self.gripper)
        print("gripper had closed!")
        time.sleep(1.2)

    def open_gripper(self,speed=255,force=255):
        # position: int[0-255], speed: int[0-255], force: int[0-255]
        self.gripper.move_and_wait_for_pos(0, speed, force)
        self.log_gripper_info(self.gripper)
        print("gripper had opened!")
        time.sleep(1.2)
    

    ## get camera data 
    def get_camera_data(self):
        color_img, depth_img = self.camera.get_data()
        return color_img, depth_img

    # Note: must be preceded by close_gripper()
    def check_grasp(self):
        # if the robot grasp object ,then the gripper is not open
        return self.get_current_tool_pos()>5

    def grasp(self,position,angle,speed=255,force=255):
        print('Executing: grasp at (%f, %f, %f) by the angle (%f, %f, %f)' \
              % (position[0], position[1], position[2],angle[0],angle[1],angle[2]))
        # if angle[angle > np.pi]:   #(-pi,pi)
        #     angle = angle - 2 * np.pi

        # Firstly, achieve pre-grasp position
        pre_position = position +[0,0,0]  # change me!
        self.move_j_p([pre_position,angle])
        # Second，achieve grasp position
        self.move_l([position,angle])
        self.close_gripper()
        if(not self.check_grasp()):
            print("Check grasp fail! ")
            pass
        # Third,put the object into box
        box_position = []  # change me!
        box_angle =[]
        self.move_j_p([box_position, box_angle])
        self.go_home()

    # def grasp1(self, position, heightmap_rotation_angle,):
    #     print('Executing: grasp at (%f, %f, %f)' % (position[0], position[1], position[2]))
    #     # Compute tool orientation from heightmap rotation angle
    #     grasp_orientation = [1.0, 0.0]
    #     if heightmap_rotation_angle > np.pi:   #(-pi,pi)
    #         heightmap_rotation_angle = heightmap_rotation_angle - 2 * np.pi
    #     tool_rotation_angle = heightmap_rotation_angle / 2
    #     tool_orientation = np.asarray(
    #         [grasp_orientation[0] * np.cos(tool_rotation_angle) - grasp_orientation[1] * np.sin(tool_rotation_angle),
    #          grasp_orientation[0] * np.sin(tool_rotation_angle) + grasp_orientation[1] * np.cos(tool_rotation_angle),
    #          0.0]) * np.pi
    #     tool_orientation_angle = np.linalg.norm(tool_orientation)
    #     tool_orientation_axis = tool_orientation / tool_orientation_angle
    #     tool_orientation_rotm = utils.angle2rotm(tool_orientation_angle, tool_orientation_axis, point=None)[:3, :3]
    #
    #     # Compute tilted tool orientation during dropping into bin
    #     tilt_rotm = utils.euler2rotm(np.asarray([-np.pi / 4, 0, 0]))
    #     tilted_tool_orientation_rotm = np.dot(tilt_rotm, tool_orientation_rotm)
    #     tilted_tool_orientation_axis_angle = utils.rotm2angle(tilted_tool_orientation_rotm)
    #     tilted_tool_orientation = tilted_tool_orientation_axis_angle[0] * np.asarray(
    #         tilted_tool_orientation_axis_angle[1:4])
    #
    #     # Attempt grasp
    #     position = np.asarray(position).copy()
    #     position[2] = max(position[2] - 0.05, workspace_limits[2][0])
    #     self.open_gripper()
    #     self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
    #     tcp_command = "def process():\n"
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" % (
    #     position[0], position[1], position[2] + 0.1, tool_orientation[0], tool_orientation[1], 0.0,
    #     self.joint_acc * 0.5, self.joint_vel * 0.5)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (
    #     position[0], position[1], position[2], tool_orientation[0], tool_orientation[1], 0.0, self.joint_acc * 0.1,
    #     self.joint_vel * 0.1)
    #     tcp_command += "end\n"
    #     self.tcp_socket.send(str.encode(tcp_command))
    #     self.tcp_socket.close()
    #     self.close_gripper()
    #
    #     # Block until robot reaches target tool position and gripper fingers have stopped moving
    #     tool_analog_input2 = self.get_current_tool_pos()
    #     timeout_t0 = time.time()
    #     while True:
    #         state_data = self.get_state()
    #         new_tool_analog_input2 = self.get_current_tool_pos()
    #         actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
    #         timeout_t1 = time.time()
    #         if (tool_analog_input2 >250 and (abs(new_tool_analog_input2 - tool_analog_input2) < 1) and all(
    #                 [np.abs(actual_tool_pose[j] - position[j]) < self.tool_pose_tolerance[j] for j in range(3)])) or (
    #                 timeout_t1 - timeout_t0) > 5:
    #             break
    #         tool_analog_input2 = new_tool_analog_input2
    #
    #     # Check if gripper is open (grasp might be successful)
    #     gripper_open = tool_analog_input2 <200   # >0.26
    #
    #     # # Check if grasp is successful
    #     # grasp_success =  tool_analog_input2 > 0.26
    #
    #     home_position = [0.49, 0.11, 0.03]
    #     bin_position = [0.5, -0.45, 0.1]
    #
    #     # If gripper is open, drop object in bin and check if grasp is successful
    #     grasp_success = False
    #     if gripper_open:
    #
    #         # Pre-compute blend radius
    #         blend_radius = min(abs(bin_position[1] - position[1]) / 2 - 0.01, 0.2)
    #
    #         # Attempt placing
    #         self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #         self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
    #         tcp_command = "def process():\n"
    #         tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=%f)\n" % (
    #         position[0], position[1], bin_position[2], tool_orientation[0], tool_orientation[1], 0.0, self.joint_acc,
    #         self.joint_vel, blend_radius)
    #         tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=%f)\n" % (
    #         bin_position[0], bin_position[1], bin_position[2], tilted_tool_orientation[0], tilted_tool_orientation[1],
    #         tilted_tool_orientation[2], self.joint_acc, self.joint_vel, blend_radius)
    #         tcp_command += "end\n"
    #         self.tcp_socket.send(str.encode(tcp_command))
    #         self.tcp_socket.close()
    #         self.open_gripper()
    #
    #         self.move_joints([home_position[0], home_position[1], home_position[2], tool_orientation[0], tool_orientation[1], 0.0])
    #         # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.0)\n" % (
    #         # home_position[0], home_position[1], home_position[2], tool_orientation[0], tool_orientation[1], 0.0,
    #         # self.joint_acc * 0.5, self.joint_vel * 0.5)
    #         #
    #         # self.tcp_socket.send(str.encode(tcp_command))
    #         # self.tcp_socket.close()
    #         # print(tcp_command) # Debug
    #
    #         # Measure gripper width until robot reaches near bin location
    #         # state_data = self.get_state()
    #         measurements = []
    #         while True:
    #             state_data = self.get_state()
    #             tool_analog_input2 = self.get_current_tool_pos()
    #             actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
    #             measurements.append(tool_analog_input2)
    #             if abs(actual_tool_pose[1] - bin_position[1]) < 0.2 or all(
    #                     [np.abs(actual_tool_pose[j] - home_position[j]) < self.tool_pose_tolerance[j] for j in
    #                      range(3)]):
    #                 break
    #
    #         # If gripper width did not change before reaching bin location, then object is in grip and grasp is successful
    #         if len(measurements) >= 2:
    #             if abs(measurements[0] - measurements[1]) < 10:
    #                 grasp_success = True
    #
    #     else:
    #         self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #         self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
    #         tcp_command = "def process():\n"
    #         tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" % (
    #         position[0], position[1], position[2] + 0.1, tool_orientation[0], tool_orientation[1], 0.0,
    #         self.joint_acc * 0.5, self.joint_vel * 0.5)
    #         tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.0)\n" % (
    #         home_position[0], home_position[1], home_position[2], tool_orientation[0], tool_orientation[1], 0.0,
    #         self.joint_acc * 0.5, self.joint_vel * 0.5)
    #         tcp_command += "end\n"
    #         self.tcp_socket.send(str.encode(tcp_command))
    #         self.tcp_socket.close()
    #
    #     # Block until robot reaches home location
    #     state_data = self.get_state()
    #     tool_analog_input2 = self.get_current_tool_pos()
    #     actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
    #     while True:
    #         state_data = self.get_state()
    #         new_tool_analog_input2 = self.get_current_tool_pos()
    #         actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
    #         if (abs(new_tool_analog_input2 - tool_analog_input2) < 1) and all(
    #                 [np.abs(actual_tool_pose[j] - home_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
    #             break
    #         tool_analog_input2 = new_tool_analog_input2
    #
    #     return grasp_success
    #
    # def push(self, position, heightmap_rotation_angle, workspace_limits):
    #     print('Executing: push at (%f, %f, %f)' % (position[0], position[1], position[2]))
    #     # Compute tool orientation from heightmap rotation angle
    #     push_orientation = [1.0, 0.0]
    #     tool_rotation_angle = heightmap_rotation_angle / 2
    #     tool_orientation = np.asarray(
    #         [push_orientation[0] * np.cos(tool_rotation_angle) - push_orientation[1] * np.sin(tool_rotation_angle),
    #          push_orientation[0] * np.sin(tool_rotation_angle) + push_orientation[1] * np.cos(tool_rotation_angle),
    #          0.0]) * np.pi
    #     tool_orientation_angle = np.linalg.norm(tool_orientation)
    #     tool_orientation_axis = tool_orientation / tool_orientation_angle
    #     tool_orientation_rotm = utils.angle2rotm(tool_orientation_angle, tool_orientation_axis, point=None)[:3, :3]
    #
    #     # Compute push direction and endpoint (push to right of rotated heightmap)
    #     push_direction = np.asarray([push_orientation[0] * np.cos(heightmap_rotation_angle) - push_orientation[
    #         1] * np.sin(heightmap_rotation_angle),
    #                                  push_orientation[0] * np.sin(heightmap_rotation_angle) + push_orientation[
    #                                      1] * np.cos(heightmap_rotation_angle), 0.0])
    #     target_x = min(max(position[0] + push_direction[0] * 0.1, workspace_limits[0][0]), workspace_limits[0][1])
    #     target_y = min(max(position[1] + push_direction[1] * 0.1, workspace_limits[1][0]), workspace_limits[1][1])
    #     push_endpoint = np.asarray([target_x, target_y, position[2]])
    #     push_direction.shape = (3, 1)
    #
    #     # Compute tilted tool orientation during push
    #     tilt_axis = np.dot(utils.euler2rotm(np.asarray([0, 0, np.pi / 2]))[:3, :3], push_direction)
    #     tilt_rotm = utils.angle2rotm(-np.pi / 8, tilt_axis, point=None)[:3, :3]
    #     tilted_tool_orientation_rotm = np.dot(tilt_rotm, tool_orientation_rotm)
    #     tilted_tool_orientation_axis_angle = utils.rotm2angle(tilted_tool_orientation_rotm)
    #     tilted_tool_orientation = tilted_tool_orientation_axis_angle[0] * np.asarray(
    #         tilted_tool_orientation_axis_angle[1:4])
    #
    #     # Push only within workspace limits
    #     position = np.asarray(position).copy()
    #     position[0] = min(max(position[0], workspace_limits[0][0]), workspace_limits[0][1])
    #     position[1] = min(max(position[1], workspace_limits[1][0]), workspace_limits[1][1])
    #     position[2] = max(position[2] + 0.005, workspace_limits[2][0] + 0.005)  # Add buffer to surface
    #
    #     home_position = [0.49, 0.11, 0.03]
    #
    #     # Attempt push
    #     self.close_gripper()
    #     self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
    #     tcp_command = "def process():\n"
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" % (
    #     position[0], position[1], position[2] + 0.1, tool_orientation[0], tool_orientation[1], tool_orientation[2],
    #     self.joint_acc * 0.5, self.joint_vel * 0.5)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (
    #     position[0], position[1], position[2], tool_orientation[0], tool_orientation[1], tool_orientation[2],
    #     self.joint_acc * 0.1, self.joint_vel * 0.1)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (
    #     push_endpoint[0], push_endpoint[1], push_endpoint[2], tilted_tool_orientation[0], tilted_tool_orientation[1],
    #     tilted_tool_orientation[2], self.joint_acc * 0.1, self.joint_vel * 0.1)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.03)\n" % (
    #     position[0], position[1], position[2] + 0.1, tool_orientation[0], tool_orientation[1], tool_orientation[2],
    #     self.joint_acc * 0.5, self.joint_vel * 0.5)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (
    #     home_position[0], home_position[1], home_position[2], tool_orientation[0], tool_orientation[1],
    #     tool_orientation[2], self.joint_acc * 0.5, self.joint_vel * 0.5)
    #     tcp_command += "end\n"
    #     self.tcp_socket.send(str.encode(tcp_command))
    #     self.tcp_socket.close()
    #
    #     # Block until robot reaches target tool position and gripper fingers have stopped moving
    #     state_data = self.get_state()
    #     while True:
    #         state_data = self.get_state()
    #         actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
    #         if all([np.abs(actual_tool_pose[j] - home_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
    #             break
    #     push_success = True
    #     time.sleep(0.5)
    #     return push_success
    #
    #
    # def restart_real(self):
    #
    #     # Compute tool orientation from heightmap rotation angle
    #     grasp_orientation = [1.0,0.0]
    #     tool_rotation_angle = -np.pi/4
    #     tool_orientation = np.asarray([grasp_orientation[0]*np.cos(tool_rotation_angle) - grasp_orientation[1]*np.sin(tool_rotation_angle), grasp_orientation[0]*np.sin(tool_rotation_angle) + grasp_orientation[1]*np.cos(tool_rotation_angle), 0.0])*np.pi
    #     tool_orientation_angle = np.linalg.norm(tool_orientation)
    #     tool_orientation_axis = tool_orientation/tool_orientation_angle
    #     tool_orientation_rotm = utils.angle2rotm(tool_orientation_angle, tool_orientation_axis, point=None)[:3,:3]
    #
    #     tilt_rotm = utils.euler2rotm(np.asarray([-np.pi/4,0,0]))
    #     tilted_tool_orientation_rotm = np.dot(tilt_rotm, tool_orientation_rotm)
    #     tilted_tool_orientation_axis_angle = utils.rotm2angle(tilted_tool_orientation_rotm)
    #     tilted_tool_orientation = tilted_tool_orientation_axis_angle[0]*np.asarray(tilted_tool_orientation_axis_angle[1:4])
    #
    #     # Move to box grabbing position
    #     box_grab_position = [0.5,-0.35,-0.12]
    #     self.open_gripper()
    #     self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
    #     tcp_command = "def process():\n"
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2]+0.1,tilted_tool_orientation[0],tilted_tool_orientation[1],tilted_tool_orientation[2],self.joint_acc,self.joint_vel)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc,self.joint_vel)
    #     tcp_command += "end\n"
    #     self.tcp_socket.send(str.encode(tcp_command))
    #     self.tcp_socket.close()
    #     self.close_gripper()
    #
    #     # Block until robot reaches box grabbing position and gripper fingers have stopped moving
    #     state_data = self.get_state()
    #     tool_analog_input2 = self.get_current_tool_pos()
    #     while True:
    #         state_data = self.get_state()
    #         new_tool_analog_input2 = self.get_current_tool_pos()
    #         actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
    #         if tool_analog_input2 >250 and (abs(new_tool_analog_input2 - tool_analog_input2) < 1) and all([np.abs(actual_tool_pose[j] - box_grab_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
    #             break  #tool_analog_input2 <3.7
    #         tool_analog_input2 = new_tool_analog_input2
    #
    #     # Move to box release position
    #     box_release_position = [0.5,0.08,-0.12]
    #     home_position = [0.49,0.11,0.03]
    #     self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
    #     tcp_command = "def process():\n"
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_release_position[0],box_release_position[1],box_release_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.1,self.joint_vel*0.1)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_release_position[0],box_release_position[1],box_release_position[2]+0.3,tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.02,self.joint_vel*0.02)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.29)\n" % (box_grab_position[0]-0.05,box_grab_position[1]+0.1,box_grab_position[2]+0.3,tilted_tool_orientation[0],tilted_tool_orientation[1],tilted_tool_orientation[2],self.joint_acc*0.5,self.joint_vel*0.5)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_grab_position[0]-0.05,box_grab_position[1]+0.1,box_grab_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.5,self.joint_vel*0.5)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.1,self.joint_vel*0.1)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_grab_position[0]+0.05,box_grab_position[1],box_grab_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.1,self.joint_vel*0.1)
    #     tcp_command += "end\n"
    #     self.tcp_socket.send(str.encode(tcp_command))
    #     self.tcp_socket.close()
    #
    #     self.open_gripper()
    #
    #     self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
    #     tcp_command = "def process():\n"
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2]+0.1,tilted_tool_orientation[0],tilted_tool_orientation[1],tilted_tool_orientation[2],self.joint_acc,self.joint_vel)
    #     tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (home_position[0],home_position[1],home_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc,self.joint_vel)
    #     tcp_command += "end\n"
    #     self.tcp_socket.send(str.encode(tcp_command))
    #     self.tcp_socket.close()
    #
    #     # Block until robot reaches home position
    #     state_data = self.get_state()
    #     tool_analog_input2 = self.get_current_tool_pos()
    #     while True:
    #         state_data = self.get_state()
    #         new_tool_analog_input2 = self.get_current_tool_pos()
    #         actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
    #         if tool_analog_input2 <200 and (abs(new_tool_analog_input2 - tool_analog_input2) < 1) and all([np.abs(actual_tool_pose[j] - home_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
    #             break  #tool_analog_input2>3
    #         tool_analog_input2 = new_tool_analog_input2
    #

if __name__ =="__main__":
    ur_robot = UR_Robot()

