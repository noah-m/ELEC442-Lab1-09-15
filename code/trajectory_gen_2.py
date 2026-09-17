import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
from pal.products.qarm import QArm

import constants


directory_path = os.path.dirname(constants.path_to_interface)
if directory_path not in sys.path:
    sys.path.append(directory_path)

from QArm_functions import QArm_Lab_interface


CONTROL_PERIOD = 0.05       # 20 Hz
KP = 1.5
ENTRY_DURATION = 10.0
SWEEP_DURATION = 5.0
NUMBER_OF_SWEEP_LEGS = 6    # B→A→B repeated three times


def follow_spline(
    spline,
    duration,
    phi_command,
    qarm_interface,
    desired_log,
    actual_log,
    label
):
    """
    Follow one task-space position spline.

    The commanded joint state is maintained between calls, ensuring
    continuity between trajectory segments.
    """

    start_time = time.perf_counter()
    previous_time = start_time
    next_report_time = 0.0

    while True:
        loop_start = time.perf_counter()

        elapsed_time = loop_start - start_time
        trajectory_time = min(elapsed_time, duration)

        dt = loop_start - previous_time
        previous_time = loop_start

        # Desired Cartesian position and velocity from the spline
        desired_position = spline(trajectory_time)
        feedforward_velocity = spline(trajectory_time, 1)

        # Use the integrated commanded state consistently for the
        # kinematic model and Jacobian.
        model_position, _ = qarm_interface.forward_kinematics(phi_command)
        J = qarm_interface.Jacobian(phi_command)[:3, :]

        model_error = desired_position - model_position

        commanded_cartesian_velocity = (
            feedforward_velocity
            + KP * model_error
        )

        joint_velocity = (
            np.linalg.pinv(J)
            @ commanded_cartesian_velocity
        )

        # Integrate the commanded joint trajectory
        phi_command = phi_command + joint_velocity * dt

        qarm_interface.write_to_arm(phi_command)

        # Measure the real/simulated arm for monitoring and plotting
        measured_phi = np.asarray(
            qarm_interface.read_from_arm(),
            dtype=float
        )

        measured_position, _ = qarm_interface.forward_kinematics(
            measured_phi
        )

        desired_log.append(np.asarray(desired_position).copy())
        actual_log.append(np.asarray(measured_position).copy())

        if trajectory_time >= next_report_time:
            actual_error = desired_position - measured_position

            print(
                f"{label}: t={trajectory_time:5.2f}, "
                f"desired y={desired_position[1]: .3f}, "
                f"actual y={measured_position[1]: .3f}, "
                f"desired z={desired_position[2]: .3f}, "
                f"actual z={measured_position[2]: .3f}, "
                f"error={np.linalg.norm(actual_error):.3f}"
            )

            next_report_time += 0.5

        if elapsed_time >= duration:
            break

        computation_time = time.perf_counter() - loop_start
        sleep_time = CONTROL_PERIOD - computation_time

        if sleep_time > 0:
            time.sleep(sleep_time)

    return phi_command


def plot_results(desired_log, actual_log):
    desired = np.asarray(desired_log)
    actual = np.asarray(actual_log)

    fig = plt.figure(figsize=(12, 5))

    ax1 = fig.add_subplot(121, projection="3d")
    ax1.plot(
        desired[:, 0],
        desired[:, 1],
        desired[:, 2],
        label="Desired",
        color="blue"
    )
    ax1.plot(
        actual[:, 0],
        actual[:, 1],
        actual[:, 2],
        label="Measured",
        color="orange"
    )
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_zlabel("z [m]")
    ax1.set_title("Task-space trajectory")
    ax1.legend()

    ax2 = fig.add_subplot(122)

    sample_number = np.arange(len(desired))

    ax2.plot(
        sample_number,
        desired[:, 2],
        label="Desired z",
        color="blue"
    )
    ax2.plot(
        sample_number,
        actual[:, 2],
        label="Measured z",
        color="orange"
    )
    ax2.set_xlabel("Control-loop sample")
    ax2.set_ylabel("z [m]")
    ax2.set_title("Vertical tracking")
    ax2.grid()
    ax2.legend()

    plt.tight_layout()
    plt.show()


def main():
    point_a = np.array([0.5, 0.4, 0.45])
    point_b = np.array([0.5, -0.4, 0.45])

    start_phi = np.zeros(4)

    qarm_interface = QArm_Lab_interface()

    mode = "-1"
    while mode not in ("0", "1"):
        mode = input("Enter 1 for real hardware, 0 for simulation: ")

    desired_log = []
    actual_log = []

    with QArm(hardware=int(mode), readMode=0) as my_arm:
        qarm_interface.attach_QArm(my_arm)

        try:
            # Move to the known starting joint configuration
            qarm_interface.write_to_arm(start_phi)
            time.sleep(4)

            phi_command = start_phi.copy()

            starting_position, _ = (
                qarm_interface.forward_kinematics(phi_command)
            )

            # ---------------------------------------------------------
            # Entry: starting position → A → B
            # ---------------------------------------------------------

            entry_points = np.array([
                starting_position,
                point_a,
                point_b
            ])

            entry_times = np.array([
                0.0,
                ENTRY_DURATION / 2,
                ENTRY_DURATION
            ])

            entry_spline = CubicSpline(
                entry_times,
                entry_points,
                axis=0,
                bc_type="clamped"
            )

            phi_command = follow_spline(
                entry_spline,
                ENTRY_DURATION,
                phi_command,
                qarm_interface,
                desired_log,
                actual_log,
                "Entry"
            )

            # The entry ends at B. Alternate B→A and A→B.
            segment_start = point_b

            for leg_number in range(NUMBER_OF_SWEEP_LEGS):
                if np.allclose(segment_start, point_b):
                    segment_end = point_a
                else:
                    segment_end = point_b

                sweep_spline = CubicSpline(
                    [0.0, SWEEP_DURATION],
                    np.array([segment_start, segment_end]),
                    axis=0,
                    bc_type="clamped"
                )

                phi_command = follow_spline(
                    sweep_spline,
                    SWEEP_DURATION,
                    phi_command,
                    qarm_interface,
                    desired_log,
                    actual_log,
                    f"Sweep {leg_number + 1}"
                )

                segment_start = segment_end

            time.sleep(2)

        finally:
            qarm_interface.shutdown()

    plot_results(desired_log, actual_log)


if __name__ == "__main__":
    main()