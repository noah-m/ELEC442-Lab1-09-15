import numpy as np

def forward_kin(phi):
    '''
    Implement a forward kinematics function for the QArm

    Input:
        phi: the joint angles in radians, in order of base, shoulder, elbow, wrist

    Output:
        location: the end effector coordinate system origin position in the base {0} coordinate system
        rotation: rotation matrix from the end effector {4} coordinate system to the base {0} coordinate system
    '''

    # Replace these two definitions
    location = None
    rotation = None

    return location, rotation

if __name__ == '__main__':
    phi = np.array([0, 0, 0, 0])
    location, rotation = forward_kin(phi)
    print(location)
    print(rotation)