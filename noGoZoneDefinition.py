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

#Computes whether or not a point is within a cube defined by 5 points: A,B,C,D,E where A,B,C,D represent the base and E is an arbitrary point that defines the height of the cube
def point_in_cube(P, A, B, C, D, E, verbose):
    # Create vectors for the base
    AB = B - A
    BC = C - B
    AC = C - A
    AD = D - A
    CD = D - C
    DA = A - D
    
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
    
    # Define the normal vectors for the six faces of the cube (pointing inwards)
    normals = [
        n_base,                     # Bottom face (ABCD)
        -n_base,                    # Top face (A_top, B_top, C_top, D_top)
        np.cross(AB, height_vector), # Side face (AB, A_top, B_top, B)
        np.cross(BC, height_vector), # Side face (AC, A_top, C_top, C)
        np.cross(AD, height_vector), # Side face (AD, A_top, D_top, D)
        np.cross(CD, height_vector)            # Opposite side face
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
            if verbose:
                print("wrong side of base")
            return False
    
    # Check top face (A_top, B_top, C_top, D_top)
    for vertex in vertices_top:
        if not point_on_correct_side(P, vertex, -1*normals[1]):
            if verbose:
                print("wrong side of top")
            return False
    
    # Check side faces
    for i in range(4):
        if not point_on_correct_side(P, vertices_base[i], -1*normals[i + 2]):
            if verbose:
                print("wrong side of the sides " + str(i+1))
            return False
    
    return True

def tip_transform(tip_offset):

    psm_T_tip = PyKDL.Frame().Identity()
    psm_T_tip.p[2] += tip_offset

    return psm_T_tip

def load_and_set_calibration():

    psm1_calib_pose = pickle.load(open("psm1_pose.p", "rb"))
    psm3_calib_pose = pickle.load(open("psm3_pose.p", "rb"))

    err = []

    psm1_tip_offset = 2.1/100
    psm3_tip_offset = 2.8/100

    psm1_T_tip = tip_transform(psm1_tip_offset)
    psm3_T_tip = tip_transform(psm3_tip_offset)

    for p in range(len(psm1_calib_pose)):

        ecm_T_psm1_tip = psm1_calib_pose[p] * psm1_T_tip
        ecm_T_psm3_tip = ecm_T_psm1_tip
        ecm_T_psm3_tip.M = psm3_calib_pose[p].M

        ecm_T_psm3_theoretical = ecm_T_psm3_tip * psm3_T_tip.Inverse() 

        offset = pm.toMatrix(ecm_T_psm3_theoretical)[0:3, 3] - pm.toMatrix(psm3_calib_pose[p])[0:3, 3]
        err.append(offset)

    offset = np.sum(np.array(err), axis=0) / len(psm1_calib_pose)
    offset_vec = PyKDL.Vector(offset[0], offset[1],  offset[2])

    return offset_vec

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
    result = point_in_cube(P, A, B, C, D, E, True)
    print("Point P is within the cube:", result)

if __name__ == '__main__':
    point_in_cube_test()
    
    print("Initializing arms...")

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")

    setting_arms_state(psm1)
    setting_arms_state(psm3)
    print("arms initialized")
    psm1_calib_pose = pickle.load(open("psm1_pose_noGo.p", "rb"))
    points = []
    for p in range(len(psm1_calib_pose)):
        points.append(pm.toMatrix(psm1_calib_pose[p])[0:3, 3])
    
    if True:
        offset = load_and_set_calibration()
        print(offset)

    while(1):
        PSM3_state =  point_in_cube(pm.toMatrix(psm3.measured_cp())[0:3,3] + np.array([offset.x(),offset.y(),offset.z()]), points[0], points[1], points[2], points[3], points[4], False) 
        PSM1_state = point_in_cube(pm.toMatrix(psm1.measured_cp())[0:3,3], points[0], points[1], points[2], points[3], points[4], True) 
        print("PSM3 = " + str(PSM3_state), "PSM1 = " + str(PSM1_state)) 
        # print(point_in_cube(pm.toMatrix(psm3.measured_cp())[0:3,3] + np.array([offset.x(),offset.y(),offset.z()]), points[0], points[1], points[2], points[3], points[4], True) )
        # print(point_in_cube(pm.toMatrix(psm1.measured_cp())[0:3,3], points[0], points[1], points[2], points[3], points[4], True) )
        # print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")