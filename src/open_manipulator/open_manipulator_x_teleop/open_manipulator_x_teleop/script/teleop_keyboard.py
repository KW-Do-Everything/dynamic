'''
#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Will Son
'''
from concurrent.futures import ThreadPoolExecutor
from math import exp
import os
import select
import sys
import rclpy

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

from open_manipulator_msgs.srv import Setarmpos, Setgripperpos, Getgripper
from baduk_msgs.msg import Vision, Point

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

g_flag = 0
# tmp =''
# tmp2 =[]

def costum_inverse(pre_pos, delta_x = 0.0, delta_y = 0.0, delta_z = 0.0, flag = 1): #반환값 세타/flage <- 기울여잡기 온 = 1 오프 = 0
    global g_pose
    global g_theta

    np.set_printoptions(precision=3, suppress=True)

    ##----------------------d----------a------alpa------theta(rad)--
    two_dof_dh_params = np.array([[0.40  , 0.      , 0.    , -pi/2 + 0.38588],
                                  [-0.0675, 0.15577 , 0     , pi/2 - 0.38588 ],
                                  [0.    , 0.138   , pi/2  , -pi/2          ]])



    dh_params = np.array([[0.40  , 0.      , 0.    , -pi/2 + 0.38588],
                          [-0.0675, 0.15577 , 0     , pi/2 - 0.38588 ],
                          [0.    , 0.138   , pi/2  , -pi/2          ],
                          [0.    , 0.06916 , 0     , 0              ],
                          [0.    , 0.06916 , 0     , 0              ]])
                        #   [0.    , 0.22051 , pi/2  , 0              ]])  #[0.05  , 0.      , 0.    , 0.             ]

    

    two_dof_robot = RobotSerial(two_dof_dh_params,"modified")
    robot_a       = RobotSerial(dh_params,"modified")

    #robot.show(body=False, ws=True)

    # =====================================
    # forward
    # =====================================
    
    const_theta = 0.38588

    s_k = np.sin(const_theta)
    c_k = np.cos(const_theta)

    #link
    L_1 = 0.15577
    L_2 = 0.13650

    # input pose (x,y) 0.18  -0.141
    x = pre_pos[0] + delta_x
    y = pre_pos[1] + delta_y
    z = pre_pos[2] + delta_z  # z>0.17, 현재는 그렇게

    if np.sqrt(x**2 + y**2) >= L_1+L_2: # 너무멀 때 세타1, 세타2
        theta_1  = np.arctan2(y,x) + pi/2 - const_theta
        theta_2 = -(pi/2 - const_theta )

    elif ((x - s_k*L_1)**2 + (y + c_k*L_1)**2) <= L_2**2: #너무 가까울 때 세타1, 세타2
        theta_1 = 0.0
        theta_2 = np.arctan2(y + c_k*L_1, x - s_k*L_1)

    else:
        # cal.. theta_2
        s_2 = (L_1**2 + L_2**2 - (x**2) - (y**2))/(2 * L_1 * L_2)
        c_2 = np.sqrt(1-s_2**2)

        theta_2 = np.arctan2(s_2,c_2) + const_theta

        #cal.. theta_1
        A = np.array([[c_2*y - s_2*x, s_2*y + c_2*x],
                    [-(s_2*y + c_2*x), c_2*y - s_2*x]])

        B = np.array([L_2 - s_2*L_1, -c_2*L_1 ])

        sol =np.linalg.solve(A,B)

        theta_1 = np.arctan2(sol[0],sol[1]) - const_theta

    print("theta_2 = ",theta_2,"[rad]", "->", theta_2*180/np.pi, "[degree]")
    print("theta_1 = ",theta_1,"[rad]", "->", theta_1*180/np.pi, "[degree]")


    # 2dof 끝
    two_theta = np.array([theta_1,theta_2 , 0.0])
    f1 = two_dof_robot.forward(two_theta)

    print("-------어깨-팔꿈치-forward-------")
    print("end frame t_4_4:")
    print(f1.t_4_4)
    
    print("--------위에서 본 좌표----------")
    three_dof_xyz = np.dot(np.linalg.inv(f1.t_4_4), np.array([x,y,z,1])) # 가져다 쓸거

    print(three_dof_xyz)
    print("end frame xyz:")
    print(f1.t_3_1.reshape([3, ]))
    print("end frame abc:")
    print(f1.euler_3)
    print("end frame rotational matrix:")
    print(f1.r_3_3)
    print("end frame quaternion:")
    print(f1.q_4)
    print("end frame angle-axis:")
    print(f1.r_3)

    # two_dof_robot.show()

    #solv 3DOF inv_kine (2자유도 끝 부터의 손목 관절 좌표계까지)

    L_3 = 0.06915
    L_4 = 0.06915

    T_x = three_dof_xyz[0]
    T_y = three_dof_xyz[1] - flag*0.228*np.sin(14.0*(pi/180))
    #cal theta_4
    
    c_4 = (T_x**2 + T_y**2 -L_3**2 - L_4**2)/(2*L_3*L_4)
    if c_4**2 >1 :
        g_theta[0] = theta_1
        g_theta[1] = theta_2
        return g_theta
    s_4 = np.sqrt(1-c_4**2) # 여기에 부호는 경로의 안(+) 밖(-)

    theta_4 = np.arctan2(s_4,c_4)

    #cal theta_3
    k_1 = L_3 + L_4*c_4
    k_2 = L_4*s_4

    theta_3 = np.arctan2(T_y,T_x) - np.arctan2(k_2, k_1)

    #cal theta_5
    theta_5 =  - theta_3 - theta_4 + flag*14.0*(pi/180)

    # 팔 전체 끝
    theta = np.array([theta_1,theta_2 , theta_3, theta_4, theta_5])
    f = robot_a.forward(theta)

    print("-------전체---forward-------")
    print("end frame t_4_4:")
    print(f.t_4_4)
    
    print("end frame xyz:")
    print(f.t_3_1.reshape([3, ]))
    print("end frame abc:")
    print(f.euler_3)
    print("end frame rotational matrix:")
    print(f.r_3_3)
    print("end frame quaternion:")
    print(f.q_4)
    print("end frame angle-axis:")
    print(f.r_3)

    print("theta:",theta)
    #robot_a.show()
    # theta_1 = round(theta_1,7)
    # theta_2 = round(theta_2,7)
    # theta_3 = round(theta_3,7)
    # theta_4 = round(theta_4,7)
    # theta_5 = round(theta_5,7)
    if flag == 1:
        theta_6 = pi/2 #np.arcsin(f.r_3_3[0,2])
    else:
        theta_6 = np.arcsin(f.r_3_3[0,2])

    print("theta: ",[theta_1,theta_2,theta_3,theta_4,theta_5,theta_6])

    g_pose=f.t_3_1.reshape([3, ])
    g_theta = [theta_1,theta_2,theta_3,theta_4,theta_5 + flag*(pi/180)*2 ,theta_6]
    return g_theta
    
usage = """
Control Your OpenManipulator!.
---------------------------
Task Space Control:
         (Forward, X+)
              W                   Q (Upward, Z+)
(Left, Y+) A     D (Right, Y-)    Z (Downward, Z-)
              X 
        (Backward, X-)

Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)
- Joint5 : Increase (V), Decrease (B)
- joint6:  Open     (G),    Close (F)

INIT : (1)
HOME : (2)

CTRL-C to quit
"""

e = """
Communications Failed
"""


class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)


    def __init__(self):
        global path_time, goal_joint_angle, present_joint_angle
        super().__init__('teleop_keyboard')
        key_value = ''

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



        # Create Service Client for Gripper
        self.gripper_action_client = self.create_client(Setgripperpos, 'set_gripper_position')
        self.gripper_action_req = Setgripperpos.Request()


        # # 그리퍼 움직임 모드와 잡혔는지 안잡혔는지 판단
        # self.gripper_moving_client = self.create_client(Getgripper, 'gripper_order')
        # self.gripper_moving_req = Setgripperpos.Request()

        # 로봇팔 서버
        # self.arm_pos_service = self.create_service(Setarmpos, 'set_arm_position', self.handle_arm_position_service)
        # self.get_logger().info("Arm position service is ready!")

        self.point_subscription = self.create_subscription(
            Point,
            'Point_topic',
            self.handle_arm_position_service,
            self.qos)
        self.point_subscription


        # 좌표 행렬
        self.coordi_table = self.make_xyz()

        # check_vision publisher
        self.check_vision_publisher = self.create_publisher(
            Vision,
            'check_vision',
            self.qos
        )

        # 초기위치 설정
        self.send_gripper_command("2") # 벌리고

        self.send_gripper_command("2")
        time.sleep(0.1)

        print(present_joint_angle)

        print_present_values()

        path_time = 2.0
        goal_joint_angle = costum_inverse([0.228, -0.23 , 0.25], flag=0) # 잡는곳으로 옮기고 [0.228 + 0.08, -0.23 , 0.28]
        self.send_goal_joint_space()
        time.sleep(path_time - 0.5)

    def check_vision_callback(self, cv: bool):
        msg = Vision()

        msg.check_vision = cv
        self.check_vision_publisher.publish(msg)

    def make_xyz(board_size = 9, board_h = 0.0236, board_w = 0.022, chun_won = np.array([0.236,0.0,0.0]),
                    err_chun_won = np.array([-0.01 - 0.004 +0.01 - 0.002, 0.01 + 0.01155 - 0.01 - 0.007,0.0])): #시작과 동시에 각 점에 해당하는 xyz 좌표를 쫙 뽑아글로벌로 저장해둠
        '''
        <read me>

        o계산에 필요한 것 
        -> base 좌표계로 부터의 천원까지의 거리(chun_won)
        -> 한 칸의 높이 (x축으로 길이) = board_h
        -> 한 칸의 폭 (y축으로 길이) = board_w
        
        o캘리브레이션에 필요한 것
        -> 천원 중심의 십자가를 그린 후 각 축의 흐트러진 정도
        -> 천원에 둘 때 실제 착점까지의 오차 [x,y]

        o그 외의 것
        -> 제어 자체를 수정(놓는데 땅에 끌리거나 닿거나)이 필요한 것들은 모션 부분에서 수정함
        '''
        #A1 = [chun_won + board_w*4 - err_chun_won[0], 0 - board_h*4 - err_chun_won[1], 0.25 + 0.0125]

        xyz = []#3차원 리스트가 될거임

        for i in range(-4,5):
            tmp = []
            for j in range(4,-5,-1):
                
                cali_matrix = np.array([[0.99396996, 0.11020809, 0.0],
                                        [-0.12117639, 0.99263099, 0.0],
                                        [0.0        , 0.0       , 1.0]])
                rota = np.array([[0.99666593, -0.08159061, 0.0],
                                [0.08159061, 0.99666593 , 0.0],
                                [0.0           ,0.0     , 1.0]])
                rota2 = np.array([[0.99939083, -0.0348995, 0],
                                  [0.0348995, 0.99939083, 0],
                                  [0.0       , 0.0       , 1]])
                tmp.append(np.dot(rota2,np.dot(rota, np.dot(cali_matrix, np.array([j, i ,1])))* np.array([board_h,board_w,0.259]) + np.array(chun_won - err_chun_won))
                           + np.array([0.0, i * 0.001, 0.0]))
             
                                    

            xyz.append(tmp)
        
        # for i in range(len(xyz[:][0])):
        #     xyz[:][i][0] += np.array([0.0, 0.01, 0.0])

        # for i in range(len(xyz[:][1])):
        #     xyz[:][i][1] += np.array([0.0, 0.0075, 0.0])
        
        # for i in range(len(xyz[:][2])):
        #     xyz[:][i][2] += np.array([0.0, 0.005, 0.0])

        # for i in range(len(xyz[:][3])):
        #     xyz[:][i][3] += np.array([0.0, 0.002, 0.0])

        return xyz
        

    def coordi_2_xyz(self, coordi): # 이미 만들어진 좌표(string)를 기준으로 바둑판 상 좌표와 매칭해서 해당하는 xyz를 리턴
        # 좌표를 문자와 숫자로 분리
        
        col, row = coordi[0], int(coordi[1:])
        
        # 'I' 문자를 건너뛰기 위한 조건 검사
        if col >= 'I':
            col_index = ord(col) - ord('A') - 1  # 'I' 이후의 문자는 하나씩 인덱스를 줄여 계산
        else:
            col_index = ord(col) - ord('A')
        
        # 행 번호를 인덱스로 변환 (0-8 범위)
        row_index = row - 1
        return self.coordi_table[col_index][ row_index]
    
    def chak_su_motion(self, xyz, ddanem): #착수 모션 수행 + 그리퍼 모션 까지 time.sleep() 이용해서
        global goal_joint_angle, g_pose, path_time, present_joint_angle,g_flag

        # stop vision
        self.check_vision_callback(False)
        
        path_time = 1.0
        goal_joint_angle = costum_inverse([0.228, -0.23 , 0.2625 - 0.026 -0.015 ], flag=g_flag) # 내리고  0.2625 - 0.026 - 0.015
        self.send_goal_joint_space()
        time.sleep(path_time)

        #온몸 비틀기
        # path_time = 1.5
        # goal_joint_angle = [0.176408, -0.799204,-0.460194,0.474000,0.254641,1.839243]
        # self.send_goal_joint_space()
        # time.sleep(path_time)


        self.send_gripper_command("1") #잡고
        self.send_gripper_command("1")
        time.sleep(0.5)

        path_time = 1.5
        goal_joint_angle = costum_inverse([0.228, -0.23 , 0.2625 ], flag=g_flag) #올리고
        self.send_goal_joint_space()
        time.sleep(path_time )

        # path_time = 1.0
        # goal_joint_angle = costum_inverse([0.228 + 0.08, -0.23 + 0.08 , 0.28 + 0.008 ]) #통에서 나올때 이동
        # self.send_goal_joint_space()
        # time.sleep(path_time)

        path_time = 2.0
        goal_joint_angle = costum_inverse(xyz, flag=g_flag) #착수위치로 이동
        self.send_goal_joint_space()
        time.sleep(path_time)

        path_time = 1.0
        goal_joint_angle = costum_inverse(xyz, delta_z = -0.02, flag=g_flag)#내리고
        self.send_goal_joint_space()
        time.sleep(path_time)

        self.send_gripper_command("2") #놓고
        self.send_gripper_command("2")
        time.sleep(0.5)

        path_time = 1.0
        goal_joint_angle = costum_inverse(xyz, delta_z = 0.02, flag=g_flag) #올리고
        ###### 수정ing
        delta_theta = np.array(goal_joint_angle) - np.array(present_joint_angle)
        # for i in range()



        self.send_goal_joint_space()
        time.sleep(path_time)

        # for i in range(1,11):
        #     path_time = 0.1
        #     goal_joint_angle = costum_inverse(g_pose, delta_z= 0.0015)
        #     self.send_goal_joint_space()
        #     time.sleep(path_time - 0.05)
        # path_time = 1.0

        

        # path_time = 2.0
        # goal_joint_angle = costum_inverse([0.228 + 0.08, -0.23 + 0.08 , 0.28 ]) #통으로 드갈때 이동
        # self.send_goal_joint_space()
        # time.sleep(path_time - 0.3)

        if ddanem != []:
            for coordi in ddanem:
                xyz = self.coordi_2_xyz(coordi)

                path_time = 2.0
                goal_joint_angle = costum_inverse(xyz, flag=0) #착수위치로 이동
                self.send_goal_joint_space()
                time.sleep(path_time)

                path_time = 1.0
                goal_joint_angle = costum_inverse(xyz, delta_z = -0.028, flag=0)#내리고
                self.send_goal_joint_space()
                time.sleep(path_time)

                self.send_gripper_command("1") #잡고
                self.send_gripper_command("1")
                time.sleep(0.5)

                path_time = 1.5
                goal_joint_angle = costum_inverse(xyz, delta_z = 0.028,flag=0) #올리고
                self.send_goal_joint_space()
                time.sleep(path_time)



                path_time = 2.0
                goal_joint_angle = costum_inverse([0.15, -0.23 , 0.25],flag=0) # 잡는곳으로 옮기고
                self.send_goal_joint_space()
                time.sleep(path_time - 0.1)

                self.send_gripper_command("2") #놓고
                self.send_gripper_command("2")
                time.sleep(0.5)

                path_time = 1.0
                goal_joint_angle = costum_inverse([0.228, -0.23 , 0.25],flag=g_flag) # 잡는곳으로 옮기고
                self.send_goal_joint_space()
                time.sleep(path_time - 0.1)

                # path_time = 2.0
                # goal_joint_angle = costum_inverse([0.228, -0.23 , 0.25], flag=0) # 잡는곳으로 옮기고
                # self.send_goal_joint_space()
                # time.sleep(path_time - 0.1)

        else:
            path_time = 2.0
            goal_joint_angle = costum_inverse([0.228, -0.23 , 0.25], flag=g_flag) # 잡는곳으로 옮기고
            self.send_goal_joint_space()
            time.sleep(path_time - 0.1)


        # motion end

        # vision restart
        self.check_vision_callback(True)
        

    def handle_arm_position_service(self, msg):
        # global tmp, tmp2

        #통으로 가라 추가해 주세요
        # self.send_gripper_command("1")

        # tmp = msg.stone_position                goal_joint_angle[-1] = pi/2

        global goal_joint_angle,g_pose, path_time
        # 입력된 좌표가 유효한지 확인하는 함수를 호출합니다.
        if self.is_valid_go_position(msg.stone_position): # 제대로된 좌표가 들어오면

            coordi = msg.stone_position

            xyz = self.coordi_2_xyz(coordi) #좌표를 base에서 본 xyz로 변환

            self.chak_su_motion(xyz, msg.minus_stone_position) # 모션 수행 // **바둑판 좌표를 xyz로 변환하는 함수를 따로 만든 것은 좌표값 자체를 수정해줘야 할 일이 많기 때문이다.
        
            '''
            # # gripper_request = Setgripperpos.Request()
            # # gripper_request.arm_move_flag = False  # 예: 그리퍼 닫기 요청

            # while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            #     self.get_logger().info('Gripper service not available, waiting...')

            # future = self.gripper_client.call_async(gripper_request) #잡아라
            # rclpy.spin_until_future_complete(self, future)  # 그리퍼 작업이 완료될 때까지 대기, 다되면 future에 데이터 들어옴

            # # goal_joint_angle = costum_inverse([0.228, -0.23 , 0.25 + 0.0125]) # 잡는곳으로 옮기고
            # # self.send_goal_joint_space()

            # # time.sleep(path_time + 0.7)

            # # gripper_request = Setgripperpos.Request()
            # # gripper_request.arm_move_flag = False  # 예: 그리퍼 닫기 요청

            # # while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            # #     self.get_logger().info('Gripper service not available, waiting...')

            # # future = self.gripper_client.call_async(gripper_request) #잡아라
            # # rclpy.spin_until_future_complete(self, future)  # 그리퍼 작업이 완료될 때까지 대기, 다되면 future에 데이터 들어옴


            # gripper_response = future.result()

            # if gripper_response.gripper_flag:
            #     self.get_logger().info('Gripper operation completed successfully.')
            #     response.arm_move_flag = True
            # else:
            #     self.get_logger().info('Failed to operate gripper.')
            #     response.arm_move_flag = False

            # # goal_joint_angle = [ 0.0, 0.0 ,pi/2, 0.0, -pi/4, 0.0]
            # # g_pose = [ 0.314, -0.141,  0.402]

            # # self.send_goal_joint_space()

            # # self.send_gripper_command("2") ##############

            # # response.arm_move_flag = True
            '''
            self.get_logger().info(f"Valid position received: {msg.stone_position}. Moving arm.")

        else:
            time.sleep(5) #5초 정지
            # self.send_goal_joint_space(False) ##### 임시
            self.get_logger().info(f"Invalid position received: {msg.stone_position}. Cannot move arm.")

        for coord in msg.minus_stone_position:
            if self.is_valid_go_position(coord):
                # 들어내기 모션
                self.get_logger().info('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            else:
                time.sleep(5) #5초 정지
                # self.send_goal_joint_space(False) ##### 임시
                # response.arm_move_flag = False
                self.get_logger().info(f"Invalid position received: {msg.stone_position}. Cannot move arm.")

    '''
###########################################################


    # def handle_arm_position_service(self, request, response):
    #     # 입력된 좌표가 유효한지 확인
    #     global goal_joint_angle,g_pose, path_time

    #     if not self.is_valid_go_position(request.stone_position):
    #         self.get_logger().info(f"Invalid position received: {request.stone_position}. Cannot move arm.")
    #         response.arm_move_flag = False
    #         goal_joint_angle = [ 0.0, 0.0 ,pi/2, 0.0, -pi/4, 0.0]
    #         g_pose = [ 0.314, -0.141,  0.402]
    #         return response

    #     # 로봇 팔 위치 설정
    #     if not self.set_arm_position(request.stone_position):
    #         response.arm_move_flag = False
    #         return response

    #     # 그리퍼 작동
    #     if not self.operate_gripper(True):  # True는 그리퍼를 닫는 것을 의미
    #         response.arm_move_flag = False
    #         return response

    #     # 모든 조작이 성공했을 때
    #     response.arm_move_flag = True
    #     self.get_logger().info("Successfully moved arm and operated gripper.")
    #     return response

    # def set_arm_position(self, position):
    #     # 로봇 팔 위치 설정 요청 생성
    #     req = SetJointPosition.Request()
    #     req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1],goal_joint_angle[2],goal_joint_angle[3],goal_joint_angle[4],goal_joint_angle[5]]
    #     req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    #     req.path_time = 2.0  # 예: 2초 동안 이동

    #     try:
    #         # 동기적 호출
    #         response = self.goal_joint_space.call(req)
    #         if response.success:
    #             self.get_logger().info("Arm position set successfully.")
    #             return True
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to set arm position: {str(e)}")
    #     return False

    # def operate_gripper(self, close):
    #     # 그리퍼 작동 요청 생성
    #     req = Setgripperpos.Request()
    #     req.close = close

    #     try:
    #         # 동기적 호출
    #         response = self.gripper_action_client.call(req)
    #         if response.success:
    #             self.get_logger().info("Gripper operated successfully.")
    #             return True
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to operate gripper: {str(e)}")
    #     return False


######################################################
    '''

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
        if row_number < 1 or row_number > 19:
            return False  # 1에서 19 범위를 벗어납니다.

        return True



    def send_gripper_command(self, gripper_mode : str): #클라이언트
        if not self.gripper_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Gripper service is not available.')
            return
        
        # self.gripper_action_req.arm_move_flag = True  # 로봇 팔이 움직임을 완료했다고 가정
        self.gripper_action_req.gripper_moving = gripper_mode # true, false

        future = self.gripper_action_client.call_async(self.gripper_action_req)   # 비동기 호출
        future.add_done_callback(self.gripper_response_callback)


        # self.get_logger().error('Let\'s go.')
        # response = self.gripper_action_client.call_async(self.gripper_action_req)  # 동기 호출
        # self.get_logger().error('jebal')

        # rclpy.spin_until_future_complete(self, response)
        # self.get_logger().error('oh yeah')



        # try:
        #     # response = self.future.result()
        #     self.gripper_response_callback(response)
        # except Exception as e:
        #     self.get_logger().error('Service call failed: %r' % (e,))


    def gripper_response_callback(self, future):
        try:
            response = future.result()
            if response.gripper_flag:
                self.get_logger().info('Gripper operation successful.')
            else:
                self.get_logger().info('Gripper operation failed.')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % e)


    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper1'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            send_goal_task = self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2','joint3','joint4','joint5','joint6'] # change
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1],goal_joint_angle[2],goal_joint_angle[3],goal_joint_angle[4],goal_joint_angle[5]] #change
        self.goal_joint_space_req.path_time = path_time

        try:
            send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    '''
    # def send_tool_control_request(self):
    #     self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2','joint3','joint4','joint5','joint6'] # change
    #     self.tool_control_req.joint_position.position =  [goal_joint_angle[0], goal_joint_angle[1],goal_joint_angle[2],goal_joint_angle[3],goal_joint_angle[4],goal_joint_angle[5]] #change
    #     self.tool_control_req.path_time = path_time

    #     try:
    #         self.tool_control_result = self.tool_control.call_async(self.tool_control_req)

    #     except Exception as e:
    #         self.get_logger().info('Tool control failed %r' % (e,))
    '''

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

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print_present_values()
    return key

def print_present_values():
    global g_theta,g_pose

    print(usage)
    print('Joint Angle(Rad): [{:.6f}, {:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]'.format(
        present_joint_angle[0],
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3],
        present_joint_angle[4],
        present_joint_angle[5],
        ))
    print("my theta:",g_theta)
    print()

    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))
    
    print("My_ pos:", g_pose)


def main():
    global goal_kinematics_pose, prev_goal_kinematics_pose, goal_joint_angle, prev_goal_joint_angle,g_pose,path_time, tmp, tmp2, g_flag
    
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboard()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):

            rclpy.spin_once(teleop_keyboard)
            # if tmp != '':
            #     teleop_keyboard.send_gripper_command("1")
            #     tmp =''
            key_value = get_key(settings)
            if key_value == 'w':
                goal_joint_angle = costum_inverse(g_pose,delta_y=-0.01,flag=0)#prev_goal_kinematics_pose

                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'x':
                goal_joint_angle = costum_inverse(g_pose,delta_y=0.01,flag=0)

                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'a':
                goal_joint_angle = costum_inverse(g_pose,delta_x=0.01,flag=0)

                teleop_keyboard.send_goal_joint_space()

            elif key_value == 'd':
                goal_joint_angle = costum_inverse(g_pose,delta_x=-0.02,flag=0)

                teleop_keyboard.send_goal_joint_space()


            elif key_value == 'q':
                goal_joint_angle = costum_inverse(g_pose,delta_z=0.01,flag=0)

                teleop_keyboard.send_goal_joint_space()


            elif key_value == 'z':
                goal_joint_angle = costum_inverse(g_pose,delta_z=-0.01,flag=0)

                teleop_keyboard.send_goal_joint_space()

            if key_value == 'n':
                teleop_keyboard.send_gripper_command("1")  # 그리퍼 잡기, gripper 노드한테 잡으라 신호 보냄
            elif key_value == 'm':
                teleop_keyboard.send_gripper_command("2")  # 그리퍼 벌리기, gripper 노드한테 펴라 신호 보냄

            #--------------------조인트 조정-------------------------------    
            elif key_value == '1':
                path_time = 0.5
                goal_joint_angle[0] += 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
                path_time = 2.0
            elif key_value == '2':
                path_time = 0.5
                goal_joint_angle[1] += 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '3':
                path_time = 0.5
                goal_joint_angle[2] +=2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '4':
                path_time = 0.5
                goal_joint_angle[3] += 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
                path_time = 2.0
            elif key_value == '5':
                path_time = 0.5
                goal_joint_angle[4] +=2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '6':
                path_time = 0.5
                goal_joint_angle[5] +=2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '!':
                path_time = 0.5
                goal_joint_angle[0] -= 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '@':
                path_time = 0.5
                goal_joint_angle[1] -= 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '#':
                path_time = 0.5
                goal_joint_angle[2]-= 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '$':
                path_time = 0.5
                goal_joint_angle[3] -= 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
                path_time = 2.0
            elif key_value == '%':
                path_time = 0.5
                goal_joint_angle[4] -=2.5*pi/180
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '^':
                path_time = 0.5
                goal_joint_angle[5] -= 2.5*pi/180
                teleop_keyboard.send_goal_joint_space()

            elif key_value == 's':

                xyz = teleop_keyboard.coordi_2_xyz('E5') #좌표를 base에서 본 xyz로 변환

                teleop_keyboard.chak_su_motion(xyz, [])
            
            elif key_value == 'S':

                xyz = teleop_keyboard.coordi_2_xyz('E5') #좌표를 base에서 본 xyz로 변환
                path_time = 2.0
                goal_joint_angle = costum_inverse(xyz,flag=0)
                teleop_keyboard.send_goal_joint_space()

            elif key_value == '0':
                if g_flag == 0:
                    g_flag = 1
                else:
                    g_flag = 0

            

            elif key_value == 'o': # 오셀로
                teleop_keyboard.send_gripper_command("1") #놓고
                teleop_keyboard.send_gripper_command("1")
                time.sleep(0.5)

                teleop_keyboard.send_gripper_command("4") #놓고
                teleop_keyboard.send_gripper_command("4")
                time.sleep(0.5)

                path_time = 3.0
                goal_joint_angle = costum_inverse([0.24, 0.107, 0.26], flag=0) #가고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                path_time = 2.0
                goal_joint_angle = costum_inverse(g_pose, delta_z=-0.03,flag=0) #내리고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                teleop_keyboard.send_gripper_command("3") #잡고
                teleop_keyboard.send_gripper_command("3")
                time.sleep(0.5)

                path_time = 2.0
                goal_joint_angle = costum_inverse(g_pose, delta_z=0.03,flag=0) #올리고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                path_time = 2.5
                goal_joint_angle = costum_inverse([0.17, 0.227, 0.25], flag=0)
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                teleop_keyboard.send_gripper_command("4") #놓고
                teleop_keyboard.send_gripper_command("4")
                time.sleep(0.5)

                path_time = 2.0
                goal_joint_angle = costum_inverse([0.2,0.227,0.25], flag=0) #따낼거로 가고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                path_time = 2.0
                goal_joint_angle =  costum_inverse(g_pose, delta_z=-0.01,flag=0)#내리고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                path_time = 2.0
                goal_joint_angle = [1.589204, -0.431049,-0.786932,0.963340,0.122718,1.555457] #온몸 비틀기
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                teleop_keyboard.send_gripper_command("3") #잡고
                teleop_keyboard.send_gripper_command("3")
                time.sleep(0.5)

                path_time = 2.0
                goal_joint_angle = [1.586136, -0.386563,-0.924990,0.302194,1.188835,1.603010]
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time-0.5)

                path_time = 2.0
                goal_joint_angle = [1.586136, -0.386563,-0.924990,0.302194,1.188835,-1.603010]
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time - 0.2)

                

                path_time = 2.0
                goal_joint_angle = [1.349903, -0.084369,-0.455592,-0.055223,1.130544,-1.584602]
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time - 0.2)

                teleop_keyboard.send_gripper_command("4") #놓고
                teleop_keyboard.send_gripper_command("4")


            elif key_value == 'c':
                path_time = 3.0
                goal_joint_angle = costum_inverse([0.19, -0.293, 0.27], flag=0)
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time - 0.3)

                path_time = 1.5
                goal_joint_angle = costum_inverse([0.25, -0.293, 0.259], delta_z=-0.01, flag=0)
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time - 0.2)

                teleop_keyboard.send_gripper_command("1") #잡고
                teleop_keyboard.send_gripper_command("1")
                time.sleep(0.5)

                path_time = 1.0
                goal_joint_angle = costum_inverse(g_pose, delta_z=0.03,flag=0) # 들고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                path_time = 0.2
                goal_joint_angle = costum_inverse([0.291, -0.244, 0.269], flag=0) # 팡 치고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time + 0.5)

                path_time = 1.0
                goal_joint_angle = costum_inverse([0.291, -0.264, 0.259], flag=0) # 내리고
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)

                teleop_keyboard.send_gripper_command("2") #잡고
                teleop_keyboard.send_gripper_command("2")
                time.sleep(0.5)

                path_time = 1.5
                goal_joint_angle = costum_inverse([0.231, -0.264, 0.259], flag=0)
                teleop_keyboard.send_goal_joint_space()
                time.sleep(path_time)


                xyz = teleop_keyboard.coordi_2_xyz('E5') #좌표를 base에서 본 xyz로 변환
                path_time = 3.0
                goal_joint_angle = costum_inverse(xyz,flag=0)
                teleop_keyboard.send_goal_joint_space()

            elif key_value == '&':

                for i in range(8,-1,-1):
                   teleop_keyboard.chak_su_motion(teleop_keyboard.coordi_table[i][4], [])
                   


            elif key_value == '*':
                for i in range(8,-1,-1):
                   teleop_keyboard.chak_su_motion(teleop_keyboard.coordi_table[4][i], [])

            elif key_value == '(':
                for i in range(9):
                   for j in range(9):
                        teleop_keyboard.chak_su_motion(teleop_keyboard.coordi_table[j][i], [])

            elif key_value == ')':
                for i in range(8,-1,-1):
                   for j in range(8,-1,-1):
                        if i == 0 or i == 8 or j == 0 or j == 8:
                            teleop_keyboard.chak_su_motion(teleop_keyboard.coordi_table[j][i], [])
                        elif j == i or j == 8 - i:
                            teleop_keyboard.chak_su_motion(teleop_keyboard.coordi_table[j][i], [])
                        elif j == 4 or i == 4:
                            teleop_keyboard.chak_su_motion(teleop_keyboard.coordi_table[j][i], [])
                        
                   
            elif key_value == '[': #앃기
                pass

            else:
                if key_value == '\x03':
                    break
                else:
                    for index in range(0, 7):
                        prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
                    for index in range(0, 6): #change
                        prev_goal_joint_angle[index] = goal_joint_angle[index]

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
