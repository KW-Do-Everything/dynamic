from concurrent.futures import ThreadPoolExecutor
from math import exp
import os
import select
import sys
import rclpy
from threading import Thread

#---------------------------------
from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi

import time
#---------------------------------

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Timer

from open_manipulator_msgs.srv import Setarmpos, Setgripperpos

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

g_theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
g_pose = [ 0.314, -0.141,  0.402]

debug = True
task_position_delta = 0.01  # meter
joint_angle_delta = 0.05  # radian
path_time = 2.8  # second


class manipulator_contoller(Node):
    
    qos = QoSProfile(depth=10)

    def __init__(self):
        super().__init__('manipulator_controller')
        
        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

        # 로봇팔 서비스 서버
        self.arm_pos_service = self.create_service(Setarmpos, 'set_arm_position', self.handle_arm_position_service)
        self.get_logger().info("Arm position service is ready!")

        # 그리퍼 서비스 클라이언트
        self.gripper_client = self.create_client(Setgripperpos, 'set_gripper_position')
        self.gripper_client_req = Setgripperpos.Request()

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4] # change
        present_joint_angle[5] = msg.position[5] # change

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 6): # change
                goal_joint_angle[index] = present_joint_angle[index]

    def handle_arm_position_service(self, request, response):
        global goal_joint_angle,g_pose, path_time
        
        self.get_logger().info(request.stone_position)

        # 두기
        if self.is_valid_go_position(request.stone_position): # 제대로된 좌표가 들어오면
            # 로봇팔이 움직이고, 다 음직였으면, 그리퍼를 움직이도록 클라이언트가 그리퍼 서버한테 요청
            self.get_logger().info("It's valid position")

            # 1. 로봇팔이 돌이 있는 위치(Home)으로 이동
            self.get_logger().info("Arm action start")

            goal_joint_angle = [0.0, pi/2, pi/2, 0.0,-pi/2, 0.0]
            g_pose = [0.057, 0.172, 0.36 ]
            done_joint = self.send_goal_joint_space()

            self.get_logger().info("Arm action finish")

            time.sleep(0.5)
            
            # 2. 로봇팔 움직임 완료
            # 그리퍼 동작 (close) - 돌 잡기
            self.send_gripper_action(True)
            self.get_logger().info("Gripper action finish") 

            time.sleep(0.5)

            # 3. 로봇팔 착수 위치로 이동
            self.get_logger().info("Arm action start")

            goal_joint_angle = [0.0, pi/2, pi/2, 0.0,-pi/2, 0.1]
            g_pose = [0.057, 0.172, 0.36 ]
            done_joint = self.send_goal_joint_space()

            self.get_logger().info("Arm action finish")

            time.sleep(0.5)

            # 4. 이동 완료
            # 그리퍼 동작 (open) - 돌 놓기
            self.send_gripper_action(False)
            self.get_logger().info("Gripper action finish")

            time.sleep(0.5)

        # 빼기
            
        return response
    
    def send_gripper_action(self, close: bool):

        self.get_logger().info("Gripper action start")
        self.gripper_client_req.gripper_moving = close
        gripper_result = self.gripper_client.call_async(self.gripper_client_req)

        # rclpy.spin_until_future_complete(self, gripper_result)

        if gripper_result.result() is not None:
            self.get_logger().info("Gripper action completed successfully")
            return gripper_result.result()
        else:
            self.get_logger().error("Failed to complete gripper action")
            return False

    def send_goal_joint_space(self):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2','joint3','joint4','joint5','joint6'] # change
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1],goal_joint_angle[2],goal_joint_angle[3],goal_joint_angle[4],goal_joint_angle[5]] #change
        self.goal_joint_space_req.path_time = path_time

        try:
            send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
            

            if send_goal_joint.result() is not None:
                self.get_logger().info('Goal Joint 성공적으로 전송됨')
            else:
                self.get_logger().error('Goal Joint 전송 실패: 서비스 호출에 실패했습니다')
    
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))
        
        return send_goal_joint.result()

    def is_valid_go_position(self, position): # 들어온 좌표가 유효한지 확인
        # 유효한 좌표는 A1~T19, I를 제외합니다.
        if len(position) < 2 or len(position) > 3:
            return False

        column = position[0].upper()
        row = position[1:]

        if column in 'I':
            return False  # 'I'는 바둑판에서 사용하지 않습니다.
        
        if column < 'A' or column > 'T':
            return False  # 'A'에서 'T' 범위를 벗어납니다 (단, 'I' 제외).

        if not row.isdigit():
            return False  # 행 숫자가 아니면 유효하지 않습니다.
        
        row_number = int(row)
        if row_number < 1 or row_number > 10:
            return False  # 1에서 19 범위를 벗어납니다.

        return True

    

def main():
    rclpy.init()

    MC = manipulator_contoller()
    rclpy.spin(MC)

    MC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()