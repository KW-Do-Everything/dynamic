#!/usr/bin/env python3

#2dof 역기구학식이 추가된 포워드 시뮬

from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi


def main():
    np.set_printoptions(precision=3, suppress=True)

    ##-------------------------------d----------a------alpa------theta(rad)--
    two_dof_dh_params = np.array([[0.40  , 0.      , 0.    , -pi/2 + 0.38588],
                                  [-0.0675, 0.15577 , 0     , pi/2 - 0.38588 ],
                                  [0.    , 0.138   , pi/2  , -pi/2          ]])


    dh_params = np.array([[0.40  , 0.      , 0.    , -pi/2 + 0.38588],
                          [-0.0675, 0.15577 , 0     , pi/2 - 0.38588 ],
                          [0.    , 0.138   , pi/2  , -pi/2          ],
                          [0.    , 0.06916 , 0     , 0              ],
                          [0.    , 0.06916 , 0     , 0              ],
                          [0.    , 0.22051 , pi/2  , 0              ]]) #[0.05  , 0.      , 0.    , 0.             ]

    

    two_dof_robot = RobotSerial(two_dof_dh_params,"modified")
    robot_a       = RobotSerial(dh_params,"modified")

    #robot.show(body=False, ws=True)

    # =====================================
    # forward
    # =====================================
    
    const_theta = 0.38264

    s_k = np.sin(const_theta)
    c_k = np.cos(const_theta)

    #link
    L_1 = 0.15577
    L_2 = 0.13650

    # input pose (x,y) 0.18  -0.141
    x = 0.236
    y = 0.0
    z = 0.23 + 0.0125 # 

    if np.sqrt(x**2 + y**2) >= L_1+L_2: # 너무멀 때 세타1, 세타2
        theta_1  = np.arctan2(y,x) + pi/2 - const_theta
        theta_2 = -(pi/2 - const_theta )

    elif ((x - s_k*L_1)**2 + (y + c_k*L_1)**2) <= L_2**2: #너무 가까울 때 세타1, 세타2
        theta_1 = 0
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
    two_theta = np.array([theta_1,theta_2 , 0])
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
    T_y = three_dof_xyz[1]
    #cal theta_4
    
    c_4 = (T_x**2 + T_y**2 -L_3**2 - L_4**2)/(2*L_3*L_4)
    s_4 = np.sqrt(1-c_4**2) # 여기에 부호는 경로의 안(+) 밖(-)

    theta_4 = np.arctan2(s_4,c_4)

    #cal theta_3
    k_1 = L_3 + L_4*c_4
    k_2 = L_4*s_4

    theta_3 = np.arctan2(T_y,T_x) - np.arctan2(k_2, k_1)

    #cal theta_5
    theta_5 =  - theta_3 - theta_4

    # 팔 전체 끝
    theta = np.array([-0.016874, -0.438719,-0.046019,1.217981,-1.204175,-0.378893])
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
    print(theta)
    robot_a.show()



if __name__ == "__main__":
    main()
