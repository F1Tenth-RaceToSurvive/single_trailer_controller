#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
import math
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.linalg import solve_continuous_are

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver
from pydrake.solvers.snopt import SnoptSolver
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import pydrake.symbolic as sym
from pydrake.autodiffutils import AutoDiffXd
from matplotlib.animation import FuncAnimation
import pdb
# sudo apt-get install python3-tk


from map import *

class Car(object):
	def __init__(self, file):
		self.prog = MathematicalProgram();

		self.world = Map(file);

		#Load a csv with x and y coordinates of the trailer path
		self.desired_trailer_path = np.loadtxt('/home/aadith/Desktop/f1_tenth/workspace/src/project/waypoints/wu_chen_obs2_spline_10.csv', delimiter=',')
		# self.desired_trailer_path = np.loadtxt('/home/aadith/Desktop/f1_tenth/workspace/src/project/waypoints/wu_chen_single_curve.csv', delimiter=',')
		# print(self.desired_trailer_path)
		self.nx = 4 # xc, yc , yawc, hitch
		self.nu = 2 # v, steering angle
		self.N = self.desired_trailer_path.shape[0]; # horizon length

		self.Q = np.diag([10,10,0,0]);
		self.R = np.diag([0,0.1]);

		# TODO :: Fill values
		self.L_trailer = 0.44;
		self.ego_to_hitch = 0.48;
		steering_angle_limit = 22;  # In degrees
		self.hitch_limit = 60;  # In degrees
		self.tracking_gain = 0.9; #The relative gains btween each tracking point
		self.acc_cost_gain = 1; #The weight of acceleration cost
		self.tracking_cost_gain = 1;#The weight of tracking cost term as a whole


		self.umax = np.array([4, steering_angle_limit*math.pi/180]);
		self.umin = np.array([0, -steering_angle_limit*math.pi/180]);
	
		self.dt_limits = np.array([0.05, 1]); # The limits of the time step
	
		
	# Write the continuous dynamics of the car system with single trailer here - on axle trailer from common road vehicles source
	def continuous_time_full_dynamics(self, x, u):
		# xc, yc, yawc, hitch
		xdot = u[0] *np.cos(x[2])
		ydot = u[0]*np.sin(x[2])
		# TODO :: Fill values -> check if correct it happens to go in the opposite direction<?>
		th_dot = u[0]*np.tan(u[1])/self.ego_to_hitch
		hitch_dot = -u[0]*(np.sin(x[3])/self.L_trailer + np.tan(u[1])/self.ego_to_hitch)
		
		f = np.array([xdot, ydot, th_dot, hitch_dot])
		return f
	
	def take_dynamics_step(self,x,u):  #Only for sim
		f = self.continuous_time_full_dynamics(x,u);
		
		x_new = np.zeros(x.shape);
		x_new[0] = x[0] + f[0]*self.dt;
		x_new[1] = x[1] + f[1]*self.dt;
		x_new[2] = x[2] + f[2]*self.dt;
		x_new[3] = x[3] + f[3]*self.dt;

		return x_new;

	def add_initial_state_constraint(self,x,x0):
		self.prog.AddBoundingBoxConstraint(x0,x0,x);

	def add_input_constraints(self, u, dt):
		for i,ui in enumerate(u):
			self.prog.AddBoundingBoxConstraint(self.umin[0], self.umax[0], ui[0])
			self.prog.AddBoundingBoxConstraint(self.umin[1], self.umax[1], ui[1])
			self.prog.AddBoundingBoxConstraint(self.dt_limits[0], self.dt_limits[1], dt[i]) # time step limit

	def add_state_constraints(self,x):
		for xi in x:
			self.prog.AddBoundingBoxConstraint(-self.hitch_limit * math.pi/180, self.hitch_limit * math.pi/180, xi[3]) # hitch angle limit
			

	def add_dynamic_constraints(self,x,u, dt):
		for i in range(self.N-1):
			f = self.continuous_time_full_dynamics(x[i,:],u[i,:])
			for j in range(self.nx):
				exp = x[i][j] + f[j]*dt[i] - x[i+1][j]
				# pdb.set_trace()
				self.prog.AddConstraint(exp[0], 0, 0);
		
	def get_trailer_position_from_car_state(self,x_car):		#Returns trailer coordinates in world frame using car state in the world frame as a 2, array
		tx_car = -self.ego_to_hitch - self.L_trailer*np.cos(x_car[3])
		ty_car = -self.L_trailer*np.sin(x_car[3])
		
		T = np.array([[np.cos(x_car[2]), -np.sin(x_car[2]), x_car[0]], 
					[np.sin(x_car[2]), np.cos(x_car[2]), x_car[1]], 
					[0, 0, 1]]);

		t = np.array([[tx_car], [ty_car], [1]]);
		t_w = np.matmul(T, t);
		t_w = t_w.flatten()[:2];
		return t_w

	def trailer_cost_evaluator(self, x):
			cost = 0;
			for i in range(x.shape[0]):
				x_current = x[i][:]
				t_w = self.get_trailer_position_from_car_state(x_current);
				cost = cost + np.linalg.norm(t_w  - self.desired_trailer_path[i,:2])**2;
				cost = self.tracking_gain * cost;
		
			return cost

	def acc_cost_evaluator(self, u):
		cost = 0;
		for i in range(u.shape[0]-1):
			acc_current = u[i][0];
			acc_next = u[i+1][0];
			cost = cost + (acc_current - acc_next)**2;
		return cost
		
	def add_costs(self, x, u):
		def trackingCostHelper(vars_x):
			car_xy = vars_x.reshape((self.N,-1))
			cost_trailer = self.trailer_cost_evaluator(car_xy)
			cost_trailer = self.tracking_cost_gain * cost_trailer;
			return cost_trailer
		
		def accCostHelper(vars_u):
			car_u = vars_u.reshape((self.N-1,-1))
			cost_acc = self.acc_cost_evaluator(car_u)
			cost_acc = self.acc_cost_gain * cost_acc;
			return cost_acc
		
		vars_x = x.flatten()
		vars_u = u.flatten()
		# Add cost for tracking trailer coordinates
		self.prog.AddCost(trackingCostHelper, vars_x);
		# Add cost for acceleration
		self.prog.AddCost(accCostHelper, vars_u);
		
	def add_obstacle_constraints(self, x):
		def obsHelper(vars):
			car_x = vars.reshape((self.N, -1))
			return self.check_collisions(car_x)
		
		vars = x.flatten()
		lb = np.zeros((self.N,), dtype = 'object')
		self.prog.AddConstraint(obsHelper,lb, lb, vars)

	def check_collisions(self, x):
		collision_states = np.zeros((self.N,), dtype='object');
		for i,xi in enumerate(x):
			xi_values = [xi[0].value(), xi[1].value(), xi[2].value(), xi[3].value()]
			if self.world.is_collided(xi_values):		#Takes times-step i, and state i and checks if has collided or not				return;
				collision_states[i] = AutoDiffXd(1);
			else:
				collision_states[i] = AutoDiffXd(0);
		return collision_states

	def add_actuation_cost(self, u):
		for ui in u:
			self.prog.AddQuadraticCost(ui @ self.R@ ui.T);

	def add_warm_start(self, x):
		for i in range(self.N):
			x_guess = np.array([self.desired_trailer_path[i,0], self.desired_trailer_path[i,1], self.desired_trailer_path[i,2], 0])
			self.prog.SetInitialGuess(x[i], x_guess )
		# for i in range(self.N-1):
		# 	self.prog.SetInitialGuess(u[i], self.u_init[i])

	def add_quadratic_cost_for_car_state(self, x):
		x_error = x - self.desired_trailer_path
		for i in range(self.N):
			self.prog.AddQuadraticCost(x_error[i] @ self.Q @ x_error[i].T)

	def compute_mpc_feedback(self):
			
		print("Computing MPC")
		
		#Inital pose of the car is at the start and trailer is straight
		x0 = np.zeros((self.nx,))
		x0[0] = self.desired_trailer_path[0,0] ;
		x0[1] = self.desired_trailer_path[0,1] ;
		x0[2] = self.desired_trailer_path[0,2];
		x0[3] = 0;

		#Initialize the decision variables
		x = np.zeros((self.N, self.nx), dtype="object")
		for i in range(self.N):
			x[i] = self.prog.NewContinuousVariables(self.nx, "x_" + str(i))

		u = np.zeros((self.N-1, self.nu), dtype="object")
		dt = np.zeros((self.N-1, ), dtype="object")
		for i in range(self.N-1):
			u[i] = self.prog.NewContinuousVariables(self.nu, "u_" + str(i))
			dt[i] = self.prog.NewContinuousVariables(1, "dt_" + str(i));

		# TODO :: Add Initial state constraint
		# self.add_initial_state_constraint(x[0], x0);

		# TODO :: Add saturation constraints
		self.add_input_constraints(u, dt);
		self.add_state_constraints(x);
		# TODO :: Add dynamics constraints
		self.add_dynamic_constraints(x,u, dt);

		# TODO :: Add obstacle constraints
		# self.add_obstacle_constraints(x);

		# TODO :: Add objective function
		self.add_costs(x, u);
		self.add_actuation_cost(u);
		#self.add_quadratic_cost_for_car_state(x);
		# TODO :: Add warm start
		self.add_warm_start(x);

		solver = SnoptSolver();
		result = solver.Solve(self.prog);
		print("MPC solution complete!")
		x_result = result.GetSolution(x);	
		# print(x_result)
		print(result.GetSolution(u))

		trailer_traj = []
		for i in range(self.N):
			x_car = x_result[i,:]
			x_trailer = self.get_trailer_position_from_car_state(x_car);
			trailer_traj += [x_trailer];
		

		self.trailer_traj = np.array(trailer_traj);
		self.car_traj = np.array(x_result[:,:2]);

		# Plot the trajectory
		fig, ax = plt.subplots()

		def animate(i):
			ax.axis('equal');
			ax.plot(self.desired_trailer_path[:,0], self.desired_trailer_path[:,1], "b--o",label = "desired path");	
			ax.plot(self.car_traj[i,0], self.car_traj[i,1],"--x" , label = "car path mpc");
			ax.plot(self.trailer_traj[i,0], self.trailer_traj[i,1], "r--o",label = "trailer path mpc");
			# Plot a the hitch
			ax.plot([self.car_traj[i,0], self.trailer_traj[i, 0]],
					[self.car_traj[i, 1], self.trailer_traj[i, 1]])

		# Call the animation
		ani = FuncAnimation(fig, animate, frames=self.N, interval=200, repeat=True)

		# Show the plot
		plt.show()








################### Testing #######################

file = "/home/aadith/Desktop/f1_tenth/workspace/src/project/maps/wu_chen_map1_obs2"
# file = "/home/aadith/Desktop/f1_tenth/workspace/src/project/maps/wu_chen_map1"
car = Car(file);
# pdb.set_trace()

# car.world.collision_test_case([0.998, 4.26]);
# car.world.collision_test_case([1, 4.17]);
# car.world.collision_test_case([1.99,-1.78]);
# car.world.collision_test_case([1.96,-1.78]);
# car.world.collision_test_case([-1.6,0.206]);
# car.world.collision_test_case([-0.845,0.654]);

car.compute_mpc_feedback();

# x0 = np.array([0,0,0,0]);
# u0 = np.array([1,-0.1]);

# car_traj = [];
# trailer_traj = []
# for i in range(0,100):
# 	car_traj += [x0];
# 	x0 = car.take_dynamics_step(x0,u0);
# 	tx_car = -car.ego_to_hitch - car.L_trailer*np.cos(x0[3])
# 	ty_car = -car.L_trailer*np.sin(x0[3])
	
# 	T = np.array([[np.cos(x0[2]), -np.sin(x0[2]), x0[0]], 
# 				[np.sin(x0[2]), np.cos(x0[2]), x0[1]], 
# 				[0, 0, 1]]);

# 	t = np.array([[tx_car], [ty_car], [1]]);
# 	t_w = np.matmul(T, t);
# 	t_w = t_w.flatten()[:2];
# 	trailer_traj += [t_w];

# #plot the trajectory
# car_traj = np.array(car_traj);
# trailer_traj = np.array(trailer_traj);
# plt.plot(car_traj[:,0],car_traj[:,1], label = "car path");
# plt.plot(trailer_traj[:,0], trailer_traj[:,1], label = "trailer path");
# plt.show();