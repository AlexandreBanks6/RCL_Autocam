import numpy as np
import dvrk
import PyKDL
import rospy
import sys
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA
import pickle
from std_msgs.msg import Bool

if sys.version_info.major < 3:
    input = raw_input
    
def pickup_transform(cam_offset):

    psm3_T_cam = PyKDL.Frame().Identity()
    psm3_T_cam.p[2] += cam_offset

    return psm3_T_cam

def flip(is_flipped):
    pub = rospy.Publisher('isflipped', Bool, queue_size=1)
    pub.publish(is_flipped)

def ring_transform(ring_offset):

    psm1_T_R = PyKDL.Frame().Identity()
    psm1_T_R.p[2] += ring_offset

    return psm1_T_R

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

#psm3_T_cam = offset from psm point to camera
#ecm_T_R = pose of the ring in the ECM coordinate frame
#ecm_T_w = world pose in frame of ECM 
def orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset):

    y_R = pm.toMatrix(ecm_T_R)[0:3, 1]

    y_R = y_R / LA.norm(y_R)
    z_i = z_i / LA.norm(z_i)
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    
    # sgn1 =  np.sign(np.dot(z_i, y_R)) 
    sgn2 = np.sign(np.dot(z_w, y_R))

    # print(sgn1 * sgn2)

    #this was causing a weird flipping of the camera? 
    # z_cam = - sgn2 * y_R 
    z_cam = -1 * y_R 

    x_cam = np.cross(z_w, z_cam) / LA.norm(np.cross(z_w, z_cam))
    y_cam = np.cross(z_cam, x_cam) / LA.norm(np.cross(z_cam, x_cam))

    x_cam_vec = PyKDL.Vector(x_cam[0], x_cam[1], x_cam[2])
    y_cam_vec = PyKDL.Vector(y_cam[0], y_cam[1], y_cam[2])
    z_cam_vec = PyKDL.Vector(z_cam[0], z_cam[1], z_cam[2])

    ecm_R_cam = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)


    ecm_p_cam = ecm_T_R.p - df * z_cam_vec

    ecm_T_cam_desired = PyKDL.Frame(ecm_R_cam, ecm_p_cam)
    ecm_T_psm3_desired = ecm_T_cam_desired * psm3_T_cam.Inverse()
    ecm_T_psm3_desired.p = ecm_T_psm3_desired.p  - offset

    return ecm_T_psm3_desired, sgn2


def compute_intermediate(psm3_T_cam, ecm_T_R, ecm_T_w, df, offset):

    x_R = pm.toMatrix(ecm_T_R)[0:3, 0]

    x_R = x_R / LA.norm(x_R)
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    sgn = np.sign(np.dot(z_w, x_R))

    z_cam =  - sgn * x_R 


    x_cam = np.cross(z_w, z_cam) / LA.norm(np.cross(z_w, z_cam))
    y_cam = np.cross(z_cam, x_cam) / LA.norm(np.cross(z_cam, x_cam))

    x_cam_vec = PyKDL.Vector(x_cam[0], x_cam[1], x_cam[2])
    y_cam_vec = PyKDL.Vector(y_cam[0], y_cam[1], y_cam[2])
    z_cam_vec = PyKDL.Vector(z_cam[0], z_cam[1], z_cam[2])

    ecm_R_cam = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)


    ecm_p_cam = ecm_T_R.p - 1.2 * df * z_cam_vec

    ecm_T_cam_desired = PyKDL.Frame(ecm_R_cam, ecm_p_cam)
    ecm_T_psm3_desired = ecm_T_cam_desired * psm3_T_cam.Inverse()
    ecm_T_psm3_desired.p = ecm_T_psm3_desired.p  - offset

    return ecm_T_psm3_desired

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

#Computes whether or not a point is below a floor defined by 5 points: A,B,C,D,E where A,B,C,D represent the base and E is an arbitrary point that defines the height of the cube
#flooroffset is the distance in meters that the "floor" is offset from face A,B,C,D along the height vector
def point_below_floor(P, A, B, C, D, E, floor_offset, verbose):
        # Create vectors for the base
    AB = B - A
    BC = C - B
    AC = C - A
    AD = D - A
    CD = D - C
    DA = A - D
    
    # Normal vector of the base plane (points inwards to the cube)
    n_base = np.cross(AC, AB) 
    n_base = n_base / np.linalg.norm(n_base)
    
    # Ensure the normal vector is pointing outwards (negative z direction)
    # if np.dot(n_base, A) > np.dot(n_base, A + np.array([0, 0, -1])):
    #     n_base = -n_base
    
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
        np.cross(BC, height_vector), # Side face (B, C, B_top, C_top)
        np.cross(CD, height_vector),            # Opposite side face
        np.cross(DA, height_vector) # Side face (AD, A_top, D_top, D)

    ]
    
    # Base and top vertices
    vertices_base = [A, B, C, D]
    vertices_top = [A_top, B_top, C_top, D_top]
    vertices_sides = [[A,B,A_top, B_top],[B,C,B_top,C_top],[C,D,C_top,D_top],[A,D,D_top,A_top]]

    width = np.linalg.norm(AB) #base dimension
    length = np.linalg.norm(AD) #base dimension
    height = np.linalg.norm(height_vector)
    #print(f"Width = {width} m, length = {length} m, height = {height} m")
    
    # Function to check if point P is on the correct side of a plane
    def point_on_correct_side(P, V, normal):
        return np.dot(P - V, normal) >= 0
    
    # Check bottom face (ABCD)
    for vertex in vertices_base:
        #note that we add the height vector to the vertices because we want to account for the size of the camera
        if not point_on_correct_side(P, vertex + floor_offset*height_vector/np.linalg.norm(height_vector),normals[0]):
            if verbose:
                print("wrong side of base")
            return True
    print("above base")
    
    return False

#PURPOSE: Given the current PSM3 and PSM1 pose, compute the pose of PSM3 that keeps PSM3 position constant but alters its orientation such that it views the centroid of PSM1
def computeSecondaryPose(currentPSM3pose, psm3_T_cam, ecm_T_R, ecm_T_w, offset):
    #keep position the same but change the rotation

    #compute vector pointing at ring from current position
    ecm_T_cam = currentPSM3pose*psm3_T_cam
    z_psm3 = (pm.toMatrix(ecm_T_R)[0:3, 3] - pm.toMatrix(ecm_T_cam)[0:3, 3] - np.array([offset.x(),offset.y(),offset.z()]) ) / LA.norm((pm.toMatrix(ecm_T_R)[0:3, 3] - pm.toMatrix(ecm_T_cam)[0:3, 3]) - np.array([offset.x(),offset.y(),offset.z()]) )
    #compute y vector
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    x_psm3 = -1 * np.cross(z_psm3,z_w)/LA.norm(np.cross(z_psm3,z_w))
    #compute x vector
    y_psm3 = np.cross(z_psm3, x_psm3)

    x_cam_vec = PyKDL.Vector(x_psm3[0], x_psm3[1], x_psm3[2])
    y_cam_vec = PyKDL.Vector(y_psm3[0], y_psm3[1], y_psm3[2])
    z_cam_vec = PyKDL.Vector(z_psm3[0], z_psm3[1], z_psm3[2])
    ecm_R_psm3 = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)
    
    secondaryPose = PyKDL.Frame(ecm_R_psm3,currentPSM3pose.p)

    return secondaryPose

## When the state is distabled, then don't run teloperation
def disabled():
    return None

## Setting arm states. We want to "enable" and "home" the two arms. Check operating_state for the two arms.

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()
        
# def initial_align(psm3, start_pose):
    
if __name__ == '__main__':

    print("Initializing arms...")
    
    rospy.init_node('dvrk_teleop', anonymous=True)

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    ecm = dvrk.ecm("ECM")

    setting_arms_state(psm1)
    setting_arms_state(psm3)
    setting_arms_state(ecm)

    ecm_pose = ecm.setpoint_cp()

    ring_offset = 0.033 ## 1.5 cm
    cam_offset = 0.045 ## 4 cm
    df = 0.12 ## in cms
    ## HARD CODED OFFSET FOR GIVEN JOINT CONFIGURATION
    
    offset = load_and_set_calibration()

    # Find respective transforms from psm1 to ring and from psm3 to cam
    psm1_T_R = ring_transform(ring_offset)
    psm3_T_cam = pickup_transform(cam_offset)    
    ecm_T_w = ecm.setpoint_cp().Inverse()

    message_rate = 0.01

    ## For first iteration, we need to gracefully park PSM3 at the start of our tracking...
    # Query the poses for psm1, psm3 and ecm
    psm1_pose = psm1.setpoint_cp()
    psm3_pose = psm3.setpoint_cp()

    ecm_T_R = psm1_pose * psm1_T_R

    z_i = pm.toMatrix(psm3_pose)[0:3, 2]

    ecm_T_psm3_desired_Pose, prev_sgn = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)
    
    print("Parking PSM3 to starting position...")
    fixed_posed = ecm_T_psm3_desired_Pose.p
    psm3.move_cp(ecm_T_psm3_desired_Pose).wait()

    input("    Press Enter to start autonomous tracking...")
    
    #gather points for noGoZone
    psm1_calib_pose = pickle.load(open("psm1_pose_noGo.p", "rb"))
    points = []
    for p in range(len(psm1_calib_pose)):
        points.append(pm.toMatrix(psm1_calib_pose[p])[0:3, 3])

    ## For every iteration:
    while not rospy.is_shutdown():

        # Query the poses for psm1, psm3 and ecm
        psm1_pose = psm1.setpoint_cp()
        psm3_pose = psm3.setpoint_cp()

        ecm_T_R = psm1_pose * psm1_T_R

        z_i = pm.toMatrix(psm3_pose)[0:3, 2]

        ecm_T_psm3_desired_Pose, curr_sgn = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)

        # if prev_sgn != curr_sgn:
        #     flip(False)
        #     ecm_T_psm3_intermediate = compute_intermediate(psm3_T_cam, ecm_T_R, ecm_T_w, df, offset)
        #     print("Flipping")
        #     psm3.move_cp(ecm_T_psm3_intermediate).wait()
        #     psm3.move_cp(ecm_T_psm3_desired_Pose).wait()
        #     # rospy.sleep(message_rate)

        # else:
        #     flip(True)
        #     psm3.move_cp(ecm_T_psm3_desired_Pose)
        #     rospy.sleep(message_rate)


        # prev_sgn = curr_sgn

        inNoGo = point_in_cube(pm.toMatrix(ecm_T_psm3_desired_Pose*psm3_T_cam)[0:3,3] + np.array([offset.x(),offset.y(),offset.z()]), points[0], points[1], points[2], points[3], points[4], verbose= False)
        belowFloor = point_below_floor(pm.toMatrix(ecm_T_psm3_desired_Pose*psm3_T_cam)[0:3,3], points[0], points[1], points[2], points[3], points[4],floor_offset=0.03, verbose= True)
       
        if (inNoGo or belowFloor):
            ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
            psm3.move_cp(ecm_T_psm3_secondaryPose)
            print("Secondary Pose below")
            print(ecm_T_psm3_secondaryPose)
            print("Primary Pose Below")
            print(psm3_pose)
            rospy.sleep(message_rate)

        else:
            psm3.move_cp(ecm_T_psm3_desired_Pose)
            rospy.sleep(message_rate)
            




## Initial Starting Position of PSM1: -4.634, -2.615, 240.000, 59.881, 13.880, -66.092   Jaw: 10.000