import dvrkKinematics
import numpy as np
import dvrk
import PyKDL
import rospy
import arm
import tf_conversions.posemath as pm
from geometry_msgs.msg import PoseStamped
import csv
import os


base_T_PSM_SUJ = []

def callback(data):
    # print("called")
    global base_T_PSM_SUJ
    p = PyKDL.Vector(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    r = PyKDL.Rotation.Quaternion(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w)
    frame = PyKDL.Frame(r,p)
    # print(frame)
    base_T_PSM_SUJ = frame

if __name__ == "__main__":
    
    rospy.init_node("AnyName")
    rospy.Rate(10000)

    PSM3 = dvrk.psm(arm_name = "PSM1")
    ECM = dvrk.ecm("ECM")
    PSM3.enable()
    PSM3.home()

    rospy.sleep(1)

    rospy.Subscriber('/PSM1/local/measured_cp', data_class= PoseStamped, callback= callback, queue_size = 1, buff_size = 1000000)

    while(not rospy.is_shutdown()):
        rospy.sleep(0.1)

        # #Compute base_T_PSM3
        # base_T_ECM = ECM.measured_cp()
        # ecm_T_PSM3 = PSM3.measured_cp()  
        
        # base_T_PSM3 = base_T_ECM*ecm_T_PSM3
        
        #create dVRK kinematic variable
        dvrk_model = dvrkKinematics.dvrkKinematics()

        #format T
        T = pm.toMatrix(base_T_PSM_SUJ)
        # print(T)

        #compute the inverse kinematics (the joint angles) to achieve the current position of PSM3
        joint = dvrk_model.ik(T)
        joint_deg = np.array(joint[0])*180/np.pi
        joint_deg[2] = joint_deg[2] * np.pi/180
        #joint_deg[5] = joint_deg[5] + 180
        #joint_deg[3] = joint_deg[3] - 90
        # print("Computed IK joints = " + str(joint_deg))

        API_joints = PSM3.measured_js()[0]
        API_joints_deg = np.array(API_joints)*180/np.pi
        API_joints_deg[2] = API_joints_deg[2] * np.pi/180

        # print("API Joints are " + str(API_joints_deg))
        print(joint_deg - API_joints_deg)
        diff=joint_deg - API_joints_deg
        with open('MotionData.csv','a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(diff.tolist()+[""]+joint_deg.tolist()+[""]+API_joints_deg.tolist())
            #writer_object.writerow(str([str(diff[0]),str(diff[1],diff[2],diff[3],diff[4],diff[5]]))
            file_object.close()
    
        #rospy.sleep(0.5)

        #ROS Topic "Local" defines wrt the robot base 
        #Generate test transform - what frame does this transform have to be in? 
        # Need to know: 
        #1. What is the input, T frame? 
        #2. What joint does this map to? 
        #3. What are the joint limits? 
        
        #Attempt various frames for comparison 

        #with respect to the base 
        
