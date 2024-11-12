import numpy as np 
import costComputation
import dVRKMotionSolver
import PyKDL
import time
import OptimizationDataLogger

## PURPOSE: To load recorded data from a ring and wire task and push it through a solver to quantify its capabilities


class solverSimulator():

    def __init__(self, filename):

        #create cost function
        self.cost = costComputation.autocamCost(kinematicsModel="Python")

        #initiliaze solver with the null conditions 
        self.cost.initializeConditions(
            q_des = np.zeros(6), 
            T_des = PyKDL.Frame(), 
            T_target = PyKDL.Frame(), 
            worldFrame= PyKDL.Frame(),
            ECM_T_PSM_RCM = PyKDL.Frame(), 
            psm3_T_cam= PyKDL.Frame(),
            distanceReg = 1.0, 
            orientationReg = 1.0, 
            similarityReg = 1.0, 
            positionReg= 1.0,
            desOrientationReg=1.0, #not in use
            desiredDistance = 0.01,
            offset=[0.0,0.0,0.0]

        )

        #initialize motion solver 
        self.motionSolver = dVRKMotionSolver.dVRKMotionSolver(
            cost_func = self.cost.computeCost,
            constraint_lb = self.cost.jointsLowerBounds,
            constraint_up = self.cost.jointsUpperBounds,
            n_joints = 6, 
            verbose = True,
            solver_iterations = 100, 
            solver_tolerance= 1e-8,
            max_solver_time=0.010
            
        )
        # self.motionSolver.prog.AddVisualizationCallback(self.cost.costCallback, self.motionSolver.q)

        self.results = {"id": [], "angleError":[], "positionError":[], "optimalCost":[], "success":[], "completionTime":[], "centroidViewAngle": [], "perpendicularAngle":[]}
        self.results_desiredPose = {"id": [], "angleError":[], "positionError":[], "optimalCost":[], "success":[], "completionTime":[], "centroidViewAngle": [], "perpendicularAngle":[]}


        self.experiment = [] #contains all of the experiment parameters and overall results

        #initialize dataloader
        self.loader = OptimizationDataLogger.OptimizationDataLogger()
        self.loader.initReading(filename)

    #PURPOSE: Steps the simulator by loading in a row, solving for joints, and recording the results 
    #REQUIRES: Number of samples in the dataset
    def stepSimulator(self, id, distanceRegArg= 1.0, orientationRegArg= 1.0, similarityRegArg= 1.0, positionRegArg= 1.0, desOrientationArg=1.0,verbose = True): 

        #READ IN ROW FROM DATA LOADER AND POPULATE SIMULATION
        i = 0
        print(str(len(self.loader.ik_indices)) + " number of IK indices" )
        time.sleep(5)
        while(1):
            print(i)
            if i > len(self.loader.ik_indices) - 1:
                break 
            success, system_time,q_des,T_des,T_target,worldFrame,ECM_T_PSM_RCM,psm3_T_cam,offset,IK_Triggered,ECM_T_PSM3 = self.loader.readDataRow(self.loader.ik_indices[i])
            if success == False:
                break
            
            if  IK_Triggered:
                #initiliaze solver with the row parameters
                self.cost.initializeConditions(
                    q_des = q_des, 
                    T_des = T_des, 
                    T_target =T_target, 
                    worldFrame= worldFrame,
                    ECM_T_PSM_RCM = ECM_T_PSM_RCM, 
                    psm3_T_cam= psm3_T_cam,
                    offset=offset,
                    distanceReg = distanceRegArg, 
                    orientationReg = orientationRegArg, 
                    similarityReg = similarityRegArg, 
                    positionReg= positionRegArg,
                    desOrientationReg=desOrientationArg, #not in use
                    desiredDistance = 0.01
                )

                #run solver
                start_time = time.time()
                success,q,optimal_cost = self.motionSolver.solve_joints(q_des)
                end_time = time.time()
                execution_time = end_time - start_time

                #compute error
                angleError, positionError = self.cost.computeArbitraryPoseError(q= q, T_1= ECM_T_PSM3)
                centroidViewAngle = np.rad2deg(np.arccos(-1 * (self.cost.centroidAngleError(self.cost.ECM_T_PSM(q)) - 1) ))
                perpendicularAngle = np.rad2deg(np.arccos(self.cost.perpendicularToFloorError(self.cost.ECM_T_PSM(q))))
                
                #save the results
                self.results["success"].append(success)
                self.results["id"].append(0)
                self.results["angleError"].append(angleError)
                self.results["positionError"].append(positionError)
                self.results["completionTime"].append(execution_time)
                self.results["optimalCost"].append(optimal_cost)
                self.results["centroidViewAngle"].append(centroidViewAngle)
                self.results["perpendicularAngle"].append(perpendicularAngle)
                

                angleError, positionError = self.cost.computeArbitraryPoseError(q= q, T_1= T_des)
                self.results_desiredPose["success"].append(success)
                self.results_desiredPose["id"].append(0)
                self.results_desiredPose["angleError"].append(angleError)
                self.results_desiredPose["positionError"].append(positionError)
                self.results_desiredPose["completionTime"].append(execution_time)
                self.results_desiredPose["optimalCost"].append(optimal_cost)
                self.results_desiredPose["centroidViewAngle"].append(centroidViewAngle)
                self.results_desiredPose["perpendicularAngle"].append(perpendicularAngle)




                if verbose:
                    print("Iter: " + str(id) + " Success: " + str(success) + " posErr: " + str(positionError) + " angErr: " + str(angleError) + " time: " + str(execution_time) + " centroidAng: " + str(centroidViewAngle) + "perpAngle: " +str(perpendicularAngle))
            i += 1

    #PURPOSE: Evaluates mean performance of the solver after a number of trials are completed
    def evaluatePerformance(self):
        performance_mean = {}
        performance_std = {}
        for key in self.results:
            mean, std = self.computeStatistics(self.results[key])
            performance_mean[key] = mean
            performance_std[key] = std
        
        self.experiment.append({"distanceReg": 0.0, "orientationReg": 0.0, "similarityReg": 0.0}) #broken feature
        print("Simulation Performance: ")
        print("mean: ", end="")
        print(performance_mean)
        print("std: ", end="")
        print(performance_std)


        performance_mean = {}
        performance_std = {}
        for key in self.results_desiredPose:
            mean, std = self.computeStatistics(self.results_desiredPose[key])
            performance_mean[key] = mean
            performance_std[key] = std


        print("Simulation Performance desiredPose: ")
        print("mean: ", end="")
        print(performance_mean)
        print("std: ", end="")
        print(performance_std)     

    def computeStatistics(self, data):
        mean_value = np.mean(data)
        std_dev =  np.std(data, ddof=1)
        return mean_value, std_dev
    
    #PURPOSE: Runs an experiment over all of the data for a given set of regularization terms
    def run_experiment(self, distanceReg, orientationReg, similarityReg, positionReg,desOrientationReg):

        #insert loop that iterates


        self.stepSimulator(id= 0, distanceRegArg=distanceReg, orientationRegArg=orientationReg, similarityRegArg=similarityReg, positionRegArg=positionReg,desOrientationArg=desOrientationReg)

        #after loop
        self.evaluatePerformance()

        


    def generate_float_range(self, start, stop, num_steps):
        """
        Generates a list of floats between two given values.

        Parameters:
            start (float): The starting value of the range.
            stop (float): The ending value of the range.
            num_steps (int): The number of steps (elements) in the range.

        Returns:
            list: A list of floats from start to stop with num_steps elements.
        """
        return np.linspace(start, stop, num_steps).tolist()




""" 
GOALS: 
1. Determine baseline of performance of the solver throughout entire trajectory
    a. Mean error (angle and postion)
    b. Completion time
    c. Number of times triggered
2. Provide tools to determine optimal configuration of weights for optimization

"""

if __name__ == "__main__":
    filename = "Data_9"
    simulator = solverSimulator(filename)
    

    simulator.run_experiment(distanceReg=10.0, orientationReg=25.0, similarityReg=1.0, positionReg=20.0,desOrientationReg=2000.0)





 