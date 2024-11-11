from motion import PSMmodel
import dvrk
import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_conversions.posemath as pm
import dVRKMotionSolver
from motion import dvrkKinematics
from motion import CmnUtil

class autocamCost():

    def __init__(self, kinematicsModel = "Python"):
        self.cost = 0
        self.kinematicsModel = kinematicsModel
        self.jointsLowerBounds = [-4.7124,-0.9250, 0.00, -4.5379, -1.3963, -1.3963]
        self.jointsUpperBounds = [ 4.7124,0.9250, 0.24,  4.5379,  1.3963,  1.3963]
        #C++ binding for da Vinci Kinematics
        if kinematicsModel == "C++":
            self.psm = PSMmodel.PSMmanipulator()
            self.psm.LoadRobot("motion/dvpsm.rob") #details the DH proximal 3 most proximal joints after SUJ
            self.psm.loadTool("PROGRASP_FORCEPS_420093")

        #Python implementation of da Vinci Kinematics 
        if kinematicsModel == "Python":
            self.psm = dvrkKinematics.dvrkKinematics()
    
    def testKinematics(self, q):
        psm_c = PSMmodel.PSMmanipulator()
        psm_c.LoadRobot("motion/dvpsm.rob") #details the DH proximal 3 most proximal joints after SUJ
        psm_c.loadTool("PROGRASP_FORCEPS_420093")

        psm_py = dvrkKinematics.dvrkKinematics()

        print("py kinematics = " + str(psm_py.fk_prograsp_420093(q)))
        print("C++ kinematics = " + str(psm_c.ForwardKinematics(q)))

        


        #q = computed joints
        #q_des = desired joints
        #T_des = desired cartesian pose in frame of the ECM
        #T_target = target that you want the distance threshold to apply to in frame of ECM
        #ECM_T_PSM_RCM = PSM's RCM in the frame of the ECM
        # xxxxxReg = regularization terms
        # desiredDistance = distance from the target 
        # worldFrame = ECM_T_W

        #variables for states 
        self.q_des = np.zeros(6)
        self.T_des = PyKDL.Frame()
        self.T_target = PyKDL.Frame()
        self.ECM_T_PSM_RCM = PyKDL.Frame()
        self.distanceReg = 1.0
        self.orientationReg = 1.0
        self.similarityReg = 1.0
        self.desiredDistance = 0.01
        self.worldFrame = PyKDL.Frame() # ECM_T_W

    #PURPOSE: updates the initial conditions to pass to the solver
    def initializeConditions(self, q_des, T_des, T_target, worldFrame, ECM_T_PSM_RCM, distanceReg = 1.0, orientationReg = 1.0, similarityReg = 1.0, desiredDistance = 0.01):
        self.q_des = q_des
        self.T_des = T_des
        self.T_target = T_target
        self.ECM_T_PSM_RCM = ECM_T_PSM_RCM
        self.distanceReg = distanceReg
        self.orientationReg = orientationReg
        self.similarityReg = similarityReg
        self.desiredDistance = desiredDistance
        self.worldFrame = worldFrame

    #Takes in ECM_T_PSM_RCM and the joints,q, to compute ECM_T_PSM: ECM_T_PSM_RCM * PSM_RCM_T_
    def ECM_T_PSM(self, q):
        if self.kinematicsModel == "C++":
            y = np.array([x.value() for x in q], dtype=np.float64)
            PSM_RCM_T_PSM = self.psm.ForwardKinematics(y)
        elif self.kinematicsModel == "Python":
            PSM_RCM_T_PSM = self.psm.fk_prograsp_420093(q)

        return pm.toMatrix(self.ECM_T_PSM_RCM) @ PSM_RCM_T_PSM

    #Assumes T is pyKDL frame
    #Computes the distance between a current pose defined by q and ECM_T_PSM_RCM and desired pose T_des and subtracts 
    #the threshold and computes the L2 norm of this
    def distanceError(self, q): 
        return np.linalg.norm( np.linalg.norm( pm.toMatrix(self.T_target)[0:3,3] - self.ECM_T_PSM(q)[0:3,3] ) - self.desiredDistance )

    #computes the L2 norm between the computed and desired joint state
    def jointSimilarity(self, q):
        return np.linalg.norm(q - self.q_des)
    
    #PURPOSE: Computes the L2 angle error between the viewing vector of the camera and the vector that views the centroid 
    def centroidAngleError(self, q):
        ECM_T_PSM = self.ECM_T_PSM(q)
        c = pm.toMatrix(self.T_target)[0:3,3] - ECM_T_PSM[0:3,3]
        z = ECM_T_PSM[0:3,2]
        return self.cosThetaBetweenTwoVectors(c,z)
    
    def perpendicularToFloorError(self, q):
        ECM_T_PSM = self.ECM_T_PSM(q)
        z_computed = ECM_T_PSM[0:3,2]
        z_w = pm.toMatrix(self.worldFrame)[0:3,2]
        x_computed = -1 * np.cross(z_computed,z_w)/np.linalg.norm(np.cross(z_computed,z_w))
        x_desired = pm.toMatrix(self.T_des)[0:3,0]
        return self.cosThetaBetweenTwoVectors(x_computed,x_desired) 
    
    def orientationError(self,q):
        return 0.5 * (self.perpendicularToFloorError(q) + self.centroidAngleError(q))
        
    def angleBetweenTwoVectors(self, a, b):
        a = a/np.linalg.norm(a)
        b = b/np.linalg.norm(b)
        return np.arccos(np.dot(a,b))
    
    def cosThetaBetweenTwoVectors(self, a, b):
        a = a/np.linalg.norm(a)
        b = b/np.linalg.norm(b)
        return 1 - np.dot(a,b)
    
    def computeCost(self, q):
        # print("Input to Cost = ", end = "")
        # print(q)
        distanceCost = self.distanceError(q)
        orientationCost= self.orientationError(q)
        similarityCost = self.jointSimilarity(q)
        costTerm = self.distanceReg*distanceCost + self.orientationReg*orientationCost + self.similarityReg*similarityCost
        return costTerm
    
    def costCallback(self, q):
        print("COST = " + str(cost.computeCost(q)))

    def computePoseError(self, q):
        T = self.ECM_T_PSM(q)
        angleErr = CmnUtil.angleError(pm.toMatrix(self.T_des), T)
        positionError = np.linalg.norm(T[0:3,3] - pm.toMatrix(self.T_des)[0:3,3])
        print("Position Error = " +str(positionError) + "m AngleErr = " + str(angleErr))
        print("joint error = " + str(q - self.q_des))


def callbackPoseStamp(data):
    # print("called")
    global base_T_PSM_SUJ
    p = PyKDL.Vector(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    r = PyKDL.Rotation.Quaternion(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w)
    frame = PyKDL.Frame(r,p)
    # print(frame)
    base_T_PSM_SUJ = frame

def callbackTransformStamp(data):
    # print("called")
    global ECM_T_PSM_SUJ
    p = PyKDL.Vector(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    r = PyKDL.Rotation.Quaternion(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w)
    frame = PyKDL.Frame(r,p)
    # print(frame)
    ECM_T_PSM_SUJ = frame

ECM_T_PSM_SUJ = PyKDL.Frame() 

if __name__ == "__main__":

    #for testing
    rospy.init_node("AnyName")
    rospy.Rate(10000)

    ARM = dvrk.psm("PSM3")
    #ECM = dvrk.ecm("ECM")
    ARM.enable()
    ARM.home()

    rospy.sleep(1)

    rospy.Subscriber('PSM3/local/measured_cp', data_class= PoseStamped, callback= callbackPoseStamp, queue_size = 1, buff_size = 1000000)
    #subscriber to obtain the pose of the RCM of PSM3 wrt the ECM 
    PSM3sub = rospy.Subscriber('SUJ/PSM3/measured_cp', data_class= TransformStamped, callback= callbackTransformStamp, queue_size = 1, buff_size = 1000000)

    rospy.sleep(1)



    q = ARM.measured_js()[0]
    q=np.array(q.tolist(),np.float64)


    ## USING THE COST FUNCTION 
    cost = autocamCost(kinematicsModel="Python")
    cost.initializeConditions(
        q_des = np.zeros(6), 
        T_des = PyKDL.Frame(), 
        T_target = PyKDL.Frame(), 
        worldFrame= PyKDL.Frame(),
        ECM_T_PSM_RCM = ECM_T_PSM_SUJ , 
        distanceReg = 1.0, 
        orientationReg = 1.0, 
        similarityReg = 1.0, 
        desiredDistance = 0.01
    )

    print("TEST RESULTS")
    # print(cost.ECM_T_PSM(q))
    # print(cost.distanceError(q))
    # print(cost.centroidAngleError(q))
    # print(cost.computeCost(q))
    cost.testKinematics(q)
    print("END OF TEST RESULTS")


    #initialize the solver
    motionSolver = dVRKMotionSolver.dVRKMotionSolver(
        cost_func = cost.computeCost,
        constraint_lb = cost.jointsLowerBounds,
        constraint_up = cost.jointsUpperBounds,
        n_joints = 6, 
        verbose = True,
        solver_iterations = 100, 
        solver_tolerance= 1e-8
    )
    motionSolver.prog.AddVisualizationCallback(cost.costCallback, motionSolver.q)
    success,q,optimal_cost = motionSolver.solve_joints(q)
    cost.computePoseError(q)