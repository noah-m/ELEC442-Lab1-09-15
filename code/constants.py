import os
from pathlib import Path


try:
    lab_machine_env = Path(os.environ["QARM_LAB_ENV"])
except KeyError as error:
    raise RuntimeError(
        "QARM_LAB_ENV is not configured on this computer."
    ) from error


controller_file = (
    lab_machine_env
    / "Lab-1"
    / "QArm_traj_controllers.py"
)

interface_file = (
    lab_machine_env
    / "QArm-control"
    / "QArm_functions.py"
)


if not controller_file.is_file():
    raise FileNotFoundError(
        f"Lab 1 controller was not found: {controller_file}"
    )

if not interface_file.is_file():
    raise FileNotFoundError(
        f"QArm interface was not found: {interface_file}"
    )


path_to_controllers = str(controller_file)
path_to_interface = str(interface_file)