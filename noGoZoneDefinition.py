import numpy as np
import dvrk
import PyKDL
import rospy
import sys
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA
import pickle

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()

    arm_pose = arm.setpoint_cp()
    arm_pose.M = PyKDL.Frame().Identity().M
    arm.move_cp(arm_pose).wait()

def point_in_cube(P, A, B, C, D, E):
    # Create vectors for the base
    AB = B - A
    AC = C - A
    AD = D - A
    
    # Normal vector of the base plane
    n_base = np.cross(AB, AC)
    n_base = n_base / np.linalg.norm(n_base)
    
    # Ensure the normal vector is pointing outwards (negative z direction)
    if np.dot(n_base, A) > np.dot(n_base, A + np.array([0, 0, -1])):
        n_base = -n_base
    
    # Height vector based on point E
    height_vector = np.dot((E - A), n_base) * n_base
    
    # Calculate the top face vertices
    A_top = A + height_vector
    B_top = B + height_vector
    C_top = C + height_vector
    D_top = D + height_vector
    
    # Define the normal vectors for the six faces of the cube (pointing outwards)
    normals = [
        n_base,                     # Bottom face (ABCD)
        -n_base,                    # Top face (A_top, B_top, C_top, D_top)
        np.cross(AB, height_vector), # Side face (AB, A_top, B_top, B)
        np.cross(AC, height_vector), # Side face (AC, A_top, C_top, C)
        np.cross(AD, height_vector), # Side face (AD, A_top, D_top, D)
        -np.cross(AB, AC)            # Opposite side face
    ]
    
    # Base and top vertices
    vertices_base = [A, B, C, D]
    vertices_top = [A_top, B_top, C_top, D_top]

    width = np.linalg.norm(AB) #base dimension
    length = np.linalg.norm(AD) #base dimension
    height = np.linalg.norm(height_vector)
    #print(f"Width = {width} m, length = {length} m, height = {height} m")
    
    # Function to check if point P is on the correct side of a plane
    def point_on_correct_side(P, V, normal):
        return np.dot(P - V, normal) <= 0
    
    # Check bottom face (ABCD)
    for vertex in vertices_base:
        if not point_on_correct_side(P, vertex, -1*normals[0]):
            print("wrong side of base")
            return False
    
    # Check top face (A_top, B_top, C_top, D_top)
    for vertex in vertices_top:
        if not point_on_correct_side(P, vertex, -1*normals[1]):
            print("wrong side of top")
            return False
    
    # Check side faces
    for i in range(4):
        if not point_on_correct_side(P, vertices_base[i], normals[i + 2]):
            print("wrong side of the sides")
            return False
    
    return True


# Example usage:
def point_in_cube_test():
    # Define points (assuming they are numpy arrays)
    A = np.array([6, 0, 0])
    B = np.array([7, 0, 0])
    C = np.array([7, 1, 0])
    D = np.array([6, 1, 0])
    E = np.array([6, 0, 2])  # E defines the height of 2 units

    # Point to check
    P = np.array([6, 0, 2.0])

    # Check if P lies within the cube
    result = point_in_cube(P, A, B, C, D, E)
    print("Point P is within the cube:", result)

if __name__ == '__main__':
    point_in_cube_test()
    
    print("Initializing arms...")

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")

    setting_arms_state(psm1)
    setting_arms_state(psm3)
    print("arms initialized")
    psm1_calib_pose = pickle.load(open("psm1_pose.p", "rb"))
    points = []
    for p in range(len(psm1_calib_pose)):
        points.append(pm.toMatrix(psm1_calib_pose[p])[0:3, 3])
    
    while(1):
        print(point_in_cube(pm.toMatrix(psm1.measured_cp())[0:3,3], points[0], points[1], points[2], points[3], points[4]) )
