from pydrake.solvers import MathematicalProgram, SolverOptions, Solve
from pydrake.solvers import IpoptSolver
import numpy as np


class dVRKMotionSolver:
    def __init__(self,cost_func,constraint_lb,constraint_up,n_joints=6,verbose=False,solver_iterations=50,solver_tolerance=1e-8):
        #cost_func of format cost_func(q), constraint_lb and constraint_up are the 
        #lower bound and uppwer bound box constraints, each np.array()'s of length n_joints
        
        #Initializes the program
        self.n_joints=n_joints
        self.prog=MathematicalProgram()
        self.q=self.prog.NewContinuousVariables(n_joints,"q") #Vars that we are solving for
        #Adds the cost
        self.cost=self.prog.AddCost(cost_func,vars=self.q) #[self.q[i] for i in range(n_joints)] self.q[:n_joints]

        #Adds the constraints of the joint limits
        self.constraints=self.prog.AddConstraint(
            self.constraint_evaluator,
            lb=constraint_lb,
            ub=constraint_up,
            vars=self.q) #[self.q[i] for i in range(n_joints)] or self.q[:n_joints]
        
        #Can also use 6 "AddBoundingBoxConstraint()" instead

        #Setting up the solver, more options here: https://coin-or.github.io/Ipopt/OPTIONS.html 
        #self.prog.SetSolverOption(IpoptSolver().solver_id(),"max_iter")
        self.solver_options=SolverOptions()
        self.solver_options.SetOption(IpoptSolver().solver_id(),"max_iter",solver_iterations)
        self.solver_options.SetOption(IpoptSolver().solver_id(),"tol",solver_tolerance)

        self.solver=IpoptSolver()

        self.verbose=verbose


    def constraint_evaluator(self,q_c):
        return np.array([q_c[i] for i in range(self.n_joints)])
    
    

    def solve_joints(self,init_joints):
        #Init joints np.array() from q0 to q1
        # print("init joints" + str(init_joints))
        # print("init joints type = " + str(type(init_joints)))
        result=self.solver.Solve(self.prog,init_joints,self.solver_options)
        success=result.is_success()
        q=result.GetSolution(self.q)
        optimal_cost=result.get_optimal_cost()

        if self.verbose:
            print("Optimization Successful: "+str(success))
            print("Optimal Cost: "+str(optimal_cost))
            print("Solution to q: "+str(q))

        #Returns
        return success,q,optimal_cost


