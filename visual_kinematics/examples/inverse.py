#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi


def main():
    np.set_printoptions(precision=3, suppress=True)

    dh_params = np.array([[0.36  , 0.      , 0.    , -pi/2 + 0.38264],
                          [0.    , 0.15222 , 0     , pi/2 - 0.38264 ],
                          [0.    , 0.12325 , pi/2  , -pi/2          ],
                          [0.    , 0.12475 , 0     , 0              ],
                          [0.    , 0.06475 , 0     , 0              ]])

    
    

    robot = RobotSerial(dh_params,"modified")


    # =====================================
    # inverse
    # =====================================

    xyz = np.array([[0.1], [0.1], [0.19]])
    abc = np.array([-pi/2, 0., 0.0])
    end = Frame.from_euler_3(abc, xyz)


    robot.inverse(end)

    print("inverse is successful: {0}".format(robot.is_reachable_inverse))
    print("axis values: \n{0}".format(robot.axis_values))
    robot.show()



if __name__ == "__main__":
    main()
