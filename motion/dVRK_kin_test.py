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
import crtk
import cisstRobotPython

### NOTE: The closed form IK appears to require the remote centre of motion for its computation. This means one must subscribe to /PSM1/local/measured_cp to obtain the PSM1 pose in the frame of the RCM

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
    

    r = cisstRobotPython.robManipulator()
    r.LoadRobot("dvpsm.rob")
    r.LoadRobot("LargeNeedleDriver.rob")

    pose = np.zeros(shape=(4,4))
    joints = np.zeros(6)
    fwd = r.ForwardKinematics(joints)

    newPose = r.InverseKinematics(joints,pose)
    print(joints)
    rospy.init_node("AnyName")
    rospy.Rate(10000)

    PSM1 = dvrk.psm("PSM1")
    #ECM = dvrk.ecm("ECM")
    PSM1.enable()
    PSM1.home()

    rospy.sleep(1)

    rospy.Subscriber('/PSM1/local/measured_cp', data_class= PoseStamped, callback= callback, queue_size = 1, buff_size = 1000000)
    
    #create dVRK kinematic variable
    dvrk_model = dvrkKinematics.dvrkKinematics()

    
    while(not rospy.is_shutdown()):
        rospy.sleep(0.1)

        # #Compute base_T_PSM3
        # base_T_ECM = ECM.measured_cp()
        # ecm_T_PSM3 = PSM3.measured_cp()  
        
        # base_T_PSM3 = base_T_ECM*ecm_T_PSM3
        


        #format T
        T = pm.toMatrix(base_T_PSM_SUJ)
        #T[0][3] = 100
        #print(T)

        #compute the inverse kinematics (the joint angles) to achieve the current position of PSM3
        joint = dvrk_model.ik(T)
        joint_deg = np.array(joint[0])*180/np.pi
        joint_deg[2] = joint_deg[2] * np.pi/180
        #joint_deg_fixed=joint_deg
        #joint_deg[5] = joint_deg[5] + 180
        #Fixes joint 4
        # if joint_deg_fixed[3]> -90:
        #     joint_deg_fixed[3] = joint_deg_fixed[3] - 90
        # elif joint_deg_fixed[3]<= -90:
        #     joint_deg_fixed[3] = joint_deg_fixed[3] + 270
        print("CloseForm IK joints = " + str(joint_deg))
        #print("Compute FK = " + str(dvrk_model.fk(np.array(joint[0]))))

        API_joints = PSM1.measured_js()[0]
        API_joints_deg = np.array(API_joints)*180/np.pi
        API_joints_deg[2] = API_joints_deg[2] * np.pi/180
        print("API Joints are " + str(np.array(API_joints_deg)))
        #print("Compute FK =  " + str(dvrk_model.fk(API_joints)))
        diff=joint_deg - API_joints_deg

        #dVRK API Inverse Kinematics Python Binding
        cisst_joints = np.zeros(6)
        API_IK = r.InverseKinematics(cisst_joints,T,1e-10,1000,0.001)
        cisst_joints_deg = np.array(cisst_joints)*180/np.pi
        cisst_joints_deg[2] = cisst_joints_deg[2] * np.pi/180
        print("Cisst Joints are " + str(cisst_joints_deg))
        print("diff = " + str(cisst_joints_deg - API_joints_deg))
        print()
        


        #PSM1.inverse_kinematics()
        with open('MotionData.csv','a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(diff.tolist()+[""]+joint_deg.tolist()+[""]+API_joints_deg.tolist())
            #writer_object.writerow(str([str(diff[0]),str(diff[1],diff[2],diff[3],diff[4],diff[5]]))
            file_object.close()
    
        #rospy.sleep(0.5)

  
