import numpy as np
from scipy.interpolate import CubicSpline
import os
import sys
import constants
from pal.products.qarm import QArm
import time

import matplotlib.pyplot as plt

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

    # 20Hz to period
    delta_t = 0.05
    locations_per_second = 1 / delta_t

    # prepend the starting position to the trajectory points
    p4, r4 = QArm_Interface.forward_kinematics(start_phi)
    trajectory_points.insert(0, p4)

    # write starting position to the arm
    QArm_Interface.write_to_arm(start_phi)
    time.sleep(4)

    # create a spline (equal knot time spacing) through the trajectory points
    knot_times = np.array([0, 5, 10])  # Define knot times for the spline
    spline = CubicSpline(knot_times, trajectory_points, axis=0, bc_type='clamped') # clamped means 1st derivative zero

    # Generate a sequence of points along the spline
    t = np.arange(0, 10 + delta_t, delta_t)  # Time values for the spline evaluation
    desired_positions = spline(t)

    #===================================================================
    points = np.asarray(trajectory_points)

    fig = plt.figure(figsize=(11, 5))

    ax1 = fig.add_subplot(121)
    ax1.plot(t, desired_positions[:, 0], label="desired x")
    ax1.plot(t, desired_positions[:, 1], label="desired y")
    ax1.plot(t, desired_positions[:, 2], label="desired z")
    ax1.scatter(knot_times, points[:, 0], marker="x")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position [m]")
    ax1.grid()
    ax1.legend()

    ax2 = fig.add_subplot(122, projection="3d")
    ax2.plot(
        desired_positions[:, 0],
        desired_positions[:, 1],
        desired_positions[:, 2],
        label="Desired spline"
    )
    ax2.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        color="red",
        label="Knot points"
    )
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_zlabel("z")
    ax2.legend()

    plt.tight_layout()
    plt.show()

    # ===============================================

    kp = 0.5
    previous_desired_position = desired_positions[0]

    phi_command = start_phi.copy()

    for desired_position in desired_positions:
        # Get current joint angles
        phi = QArm_Interface.read_from_arm()

        # Compute the Jacobian at the current joint angles
        J = QArm_Interface.Jacobian(phi_command)
        #reshape jacobian to 3x4
        J = J[:3, :4]

        # Compute the desired end-effector velocity (simple proportional control)
        p4, _ = QArm_Interface.forward_kinematics(phi_command)
        error = desired_position - p4
        delta_desired_position = desired_position - previous_desired_position
        desired_velocity = delta_desired_position / delta_t + kp * error
        previous_desired_position = desired_position

        # Compute the required joint velocities using the Jacobian
        try:
            joint_velocities = np.linalg.pinv(J) @ desired_velocity
        except np.linalg.LinAlgError:
            print("Jacobian is singular, cannot compute joint velocities.")
            continue

        # Update joint angles based on computed velocities
        phi_command = phi_command + joint_velocities * delta_t

        # Write new joint angles to the arm
        QArm_Interface.write_to_arm(phi_command)

        print(f"Desired Position: {desired_position}, Current Position: {p4}, Error: {error}")
        print(f"Error Norm: {np.linalg.norm(error)}")
        # Wait for the next time step
        time.sleep(delta_t)

    # ===================================================================

    

    #! convert to a starting joint value
    point_a = np.array([0.5, 0.4, 0.45])
    point_b = np.array([0.5, -0.4, 0.45])
    trajectory_points = [
        # [X, Y, Z]
        point_b,
        point_a,
        point_b,
        point_a,
        point_b,
        point_a
    ]

    knot_times = np.array([0, 5, 10, 15, 20, 25])  # Define knot times for the spline
    spline = CubicSpline(knot_times, trajectory_points, axis=0, bc_type='clamped') # clamped means 1st derivative zero

    # Generate a sequence of points along the spline
    t = np.arange(0, 25 + delta_t, delta_t)  # Time values for the spline evaluation
    desired_positions = spline(t)

    #===================================================================
    points = np.asarray(trajectory_points)

    fig = plt.figure(figsize=(11, 5))

    ax1 = fig.add_subplot(121)
    ax1.plot(t, desired_positions[:, 0], label="desired x")
    ax1.plot(t, desired_positions[:, 1], label="desired y")
    ax1.plot(t, desired_positions[:, 2], label="desired z")
    ax1.scatter(knot_times, points[:, 0], marker="x")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position [m]")
    ax1.grid()
    ax1.legend()

    ax2 = fig.add_subplot(122, projection="3d")
    ax2.plot(
        desired_positions[:, 0],
        desired_positions[:, 1],
        desired_positions[:, 2],
        label="Desired spline"
    )
    ax2.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        color="red",
        label="Knot points"
    )
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_zlabel("z")
    ax2.legend()

    plt.tight_layout()
    plt.show()

    # ===============================================

    kp = 0.5
    previous_desired_position = desired_positions[0]

    for desired_position in desired_positions:
        # Get current joint angles
        phi = QArm_Interface.read_from_arm()

        # Compute the Jacobian at the current joint angles
        J = QArm_Interface.Jacobian(phi_command)
        #reshape jacobian to 3x4
        J = J[:3, :4]

        # Compute the desired end-effector velocity (simple proportional control)
        p4, _ = QArm_Interface.forward_kinematics(phi_command)
        error = desired_position - p4
        delta_desired_position = desired_position - previous_desired_position
        desired_velocity = delta_desired_position / delta_t + kp * error
        previous_desired_position = desired_position

        # Compute the required joint velocities using the Jacobian
        try:
            joint_velocities = np.linalg.pinv(J) @ desired_velocity
        except np.linalg.LinAlgError:
            print("Jacobian is singular, cannot compute joint velocities.")
            continue

        # Update joint angles based on computed velocities
        phi_command = phi_command + joint_velocities * delta_t

        # Write new joint angles to the arm
        QArm_Interface.write_to_arm(phi_command)

        print(f"Desired Position: {desired_position}, Current Position: {p4}, Error: {error}")
        print(f"Error Norm: {np.linalg.norm(error)}")

        # Wait for the next time step
        time.sleep(delta_t)

    time.sleep(4)