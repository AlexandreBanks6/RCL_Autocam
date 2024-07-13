import numpy as np
import dvrk
import PyKDL
import rospy
import sys
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA
import pickle

if sys.version_info.major < 3:
    input = raw_input

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()

    arm_pose = arm.setpoint_cp()
    arm_pose.M = PyKDL.Frame().Identity().M
    arm.move_cp(arm_pose).wait()


def collectNoGo(psm1):


    # psm1.move_cp(psm1_Identity_pose).wait()
    # psm3.move_cp(psm3_Identity_pose).wait()
    psm1_pose = []

    n = 5
    print(f"Now collecting {n} points...")
    for i in range(n):

        print(" Please move PSM1 to touch each fiducial.")
        input("    Press Enter to continue...")
        psm1_pose.append(psm1.measured_cp())

    print("Finished Calibration for PSM1")


    pickle.dump(psm1_pose, open("psm1_pose_noGo.p", "wb")) 

if __name__ == '__main__':

    print("Initializing arm...")

    psm1 = dvrk.psm("PSM1")

    setting_arms_state(psm1)

    collectNoGo(psm1)





