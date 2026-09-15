import numpy as np
from scipy.interpolate import CubicSpline
import os
import sys
import constants
from pal.products.qarm import QArm
import time

directory_path = os.path.dirname(constants.path_to_interface)
if directory_path not in sys.path:
    sys.path.append(directory_path)
from QArm_functions import QArm_Lab_interface


#! convert to a starting joint value
trajectory_points = [
    # [X, Y, Z]
    [0.5, 0.4, 0.45],
    [0.5, -0.4, 0.45]
]

start_phi = np.array([0.0, 0.0, 0.0, 0.0])
QArm_Interface = QArm_Lab_interface()


mode = "-1"
while(int(mode) != 0 and int(mode) != 1):
    mode = input("Enter 1 for real hardware, 0 for simulation: ")

with QArm(hardware=int(mode), readMode=0) as myArm:

    QArm_Interface.attach_QArm(myArm)

    #TODO: Travel back and forth between the waypoints using cublic spline trajectory generation through task space and differential kinematics (use knot points and forward kin to deal with error)
    
    # Use the following helper functions
    # QArm_Interface.write_to_arm(joint_positions)
    # phi = QArm_Interface.read_from_arm()
    # J = QArm_Interface.Jacobian(phi)
    # p4, _ = QArm_Interface.forward_kinematics(phi)

    time.sleep(4)