import os
import sys
import constants

directory_path = os.path.dirname(constants.path_to_controllers)

if directory_path not in sys.path:
    sys.path.append(directory_path)

from QArm_traj_controllers import CartesianJacobianController, JointSpaceController, LetterTrajectoryController


def main():
    # Define arbitrary waypoints: [X, Y, Z, Target Time (seconds)]
    # Note: For the Jacobian Spline to work properly, time MUST strictly increase.
    # Feel free to add or remove points as you see fit, note that if it violates joint or workspace limits, the script will terminate
    trajectory_points = [
        [0.05, 0.0, 0.8, 0.0],
        [0.0, 0.7, 0.35, 6.0],
        [0.7, 0.0, 0.2, 12.0],
        [-0.5, -0.25, 0.3, 18.0]
    ]

    # trajectory_points = [
    #     [0.1, 0.0, 0.80, 0.0],
    #     [0.73, 0.0, 0.09, 6.0]
    # ]

    #! Check for waypoints and trajectories outside workspace, but esspeically in the ground

    mode = "-1"
    while(int(mode) != 0 and int(mode) != 1):
        mode = input("Enter 1 for real hardware, 0 for simulation: ")

    print("Select Controller:")
    print("1: Cartesian Spline (Smooth, Jacobian-based)")
    print("2: Joint Space (Point-to-Point, IK-based)")
    
    choice = input("Enter 1 or 2: ")

    if choice == '1':
        print("\nStarting Cartesian Controller...")
        controller = CartesianJacobianController(trajectory_points, int(mode))
    elif choice == '2':
        print("\nStarting Joint Space Controller...")
        controller = JointSpaceController(trajectory_points, int(mode))
    elif choice == '3':
        print("\nStarting Letter Controller...")
        initials = input("Enter 2 letters: ")
        controller = LetterTrajectoryController(initials, int(mode))

    # This handles the hardware connection, 3D plotting, and trajectory loop automatically
    controller.run()

if __name__ == "__main__":
    main()