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
from pydrake.solvers.ipopt import IpoptSolver
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import pydrake.symbolic as sym
from pydrake.autodiffutils import AutoDiffXd
from matplotlib.animation import FuncAnimation
import pdb

from map import *

class Car(object):
	def __init__(self, file):
		self.prog = MathematicalProgram();

		self.world = Map(file);

		#Load a csv with x and y coordinates of the trailer path
		self.desired_trailer_path = np.loadtxt('../waypoints/wu_chen_2_obs2_spline_10.csv', delimiter=',')
		# self.desired_trailer_path = np.loadtxt('/home/aadith/Desktop/f1_tenth/workspace/src/project/waypoints/wu_chen_single_curve.csv', delimiter=',')
		self.nx = 4 # xc, yc , yawc, hitch
		self.nu = 2 # v, steering angle
		self.N = 50; # horizon length

		self.Q = np.diag([10,10,0,0]);
		self.R = np.diag([0,0.1]);

		# TODO: Parameters for the car-trailer system
		self.L_trailer = 0.44;
		self.ego_to_hitch = 0.48;
		steering_angle_limit = 22;  # In degrees
		self.hitch_limit = 40;  # In degrees

		self.tracking_gain = 1 #0.9; #The relative gains btween each tracking point

		self.acc_cost_gain = 1; #The weight of acceleration cost
		self.tracking_cost_gain = 15;#The weight of tracking cost term as a whole
		self.final_tracking_gain = 15; # final tracking cost gain for the final position of the trailer

		# TODO: Constriants limits for the inputs and variables
		self.umax = np.array([3, steering_angle_limit*math.pi/180]);
		self.umin = np.array([0, -steering_angle_limit*math.pi/180]);
		self.dt_limits = np.array([0.05, 0.15]); # The limits of the time step
	
		# self.map_min = np.array([-3.9, -4.8])
		# self.map_max = np.array([4.5, 4.2])

		self.occupied_pixels = []
		for row in range(self.world.map.shape[0]):
			for col in range(self.world.map.shape[1]):
				if self.world.map[row, col] == 0:
					x_world, y_world = self.world.pixel_to_world(row, col)
					self.occupied_pixels.append([x_world, y_world])
		self.occupied_pixels = np.asarray(self.occupied_pixels)
		
	# function for continuous dynamics of the car system with single trailer here
	def continuous_time_full_dynamics(self, x, u):
		# states: xc, yc, yawc, hitch
		xdot = u[0] *np.cos(x[2])
		ydot = u[0]*np.sin(x[2])
		th_dot = u[0]*np.tan(u[1])/self.ego_to_hitch
		hitch_dot = -u[0]*(np.sin(x[3])/self.L_trailer + np.tan(u[1])/self.ego_to_hitch)
		
		f = np.array([xdot, ydot, th_dot, hitch_dot])
		return f
	
	# TESTING FUNCTION: to test the dynamics function
	def take_dynamics_step(self,x,u): 
		f = self.continuous_time_full_dynamics(x,u);
		x_new = np.zeros(x.shape);
		x_new[0] = x[0] + f[0]*self.dt;
		x_new[1] = x[1] + f[1]*self.dt;
		x_new[2] = x[2] + f[2]*self.dt;
		x_new[3] = x[3] + f[3]*self.dt;

		return x_new;

	def add_initial_state_constraint(self,x,x0):
		# self.prog.AddBoundingBoxConstraint(x0,x0,x);
		self.prog.AddLinearEqualityConstraint(x,x0);

	def add_input_constraints(self, u, dt):
		for i,ui in enumerate(u):
			self.prog.AddBoundingBoxConstraint(self.umin[0], self.umax[0], ui[0])
			self.prog.AddBoundingBoxConstraint(self.umin[1], self.umax[1], ui[1])
			self.prog.AddBoundingBoxConstraint(self.dt_limits[0], self.dt_limits[1], dt[i]) # time step limit

	def add_state_constraints(self,x):
		for xi in x:
			self.prog.AddBoundingBoxConstraint(-self.hitch_limit * math.pi/180, self.hitch_limit * math.pi/180, xi[3]) # hitch angle limit
			# self.prog.AddBoundingBoxConstraint(self.map_min[0], self.map_max[0], xi[0])
			# self.prog.AddBoundingBoxConstraint(self.map_min[1], self.map_max[1], xi[1])


	def add_dynamic_constraints(self,x,u, dt):
		for i in range(self.N-1):
			f = self.continuous_time_full_dynamics(x[i,:],u[i,:])
			for j in range(self.nx):
				exp = x[i][j] + f[j]*dt[i] - x[i+1][j]
				self.prog.AddConstraint(exp[0], 0, 0);
		
	def get_trailer_position_from_car_state(self,x_car):		
		#Returns trailer coordinates in world frame using car state in the world frame as a 2, array
		tx_car = -self.ego_to_hitch - self.L_trailer*np.cos(x_car[3])
		ty_car = -self.L_trailer*np.sin(x_car[3])
		
		T = np.array([[np.cos(x_car[2]), -np.sin(x_car[2]), x_car[0]], 
					[np.sin(x_car[2]), np.cos(x_car[2]), x_car[1]], 
					[0, 0, 1]]);

		t = np.array([[tx_car], [ty_car], [1]]);
		t_w = np.matmul(T, t);
		t_w = t_w.flatten()[:2];
		return t_w

	def find_closest_point_cost(self, xi, ref):
		#Find the closest point on the path to the current state
		dist = np.sum((ref - xi[:2])**4, axis = 1);
		closest_dist = np.min(dist);
		return closest_dist
	
	def trailer_cost_evaluator(self, x):
			cost = 0;
			for i in range(x.shape[0]-1):
				x_current = x[i][:]
				t_w = self.get_trailer_position_from_car_state(x_current);
				dist_cost = self.find_closest_point_cost(t_w, self.desired_trailer_path[:,:2])
				cost = cost + self.tracking_gain*dist_cost;
				# cost = self.tracking_gain * cost;
			
			t_w_final = self.get_trailer_position_from_car_state(x[-1]);
			final_trailer_cost = np.sum((t_w_final - self.desired_trailer_path[-1,:2])**2);
			cost = cost + self.final_tracking_gain * final_trailer_cost;
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
			print("cost_trailer: ", cost_trailer)
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

	def add_warm_start(self, x, u, dt):
		# x_guess = np.linspace([self.desired_trailer_path[0,0], self.desired_trailer_path[0,1], self.desired_trailer_path[0,3], 0],\
		# 	 				   [self.desired_trailer_path[-1,0], self.desired_trailer_path[-1,1], self.desired_trailer_path[-1,3], 0], self.N)
		
		# u0_guess = np.random.choice(np.linspace(self.umin[0], self.umax[0], self.N-1), self.N-1).reshape(-1,1)
		# u1_guess = np.random.choice(np.linspace(self.umin[1], self.umax[1], self.N-1), self.N-1).reshape(-1,1)
		# u_guess = np.hstack((u0_guess, u1_guess))

		# dt_guess = np.random.choice(np.linspace(self.dt_limits[0], self.dt_limits[1], self.N-1), self.N-1)


		# # for i in range(self.N):
		# # 	self.prog.SetInitialGuess(x[i], x_guess[i])
		# for i in range(self.N-1):
		# 	self.prog.SetInitialGuess(u[i], u_guess[i])
		# 	self.prog.SetInitialGuess(dt[i][0], dt_guess[i])


		x_guess_unit = (self.desired_trailer_path[0,:] - self.desired_trailer_path[-1,:])/self.N
		for i in range(self.N):
			x_guess = self.desired_trailer_path[0,:] - i*x_guess_unit
			x_guess[2] = 0
			x_guess[3] = 0
			self.prog.SetInitialGuess(x[i], x_guess)

		# for i in range(self.N-1):
		# 	self.prog.SetInitialGuess(u[i], self.u_init[i])

	# def add_quadratic_cost_for_car_state(self, x):
	# 	# TESTING FUNCTION - only used for testing the mpc code
	# 	x_error = x - self.desired_trailer_path
	# 	for i in range(self.N):
	# 		self.prog.AddQuadraticCost(x_error[i] @ self.Q @ x_error[i].T)

	def compute_mpc_feedback(self):
			
		print("Computing MPC")
		
		#Inital pose of the car is at the start and trailer is straight
		x0 = np.zeros((self.nx,))
		x0[0] = np.cos(self.desired_trailer_path[0,3])*(self.ego_to_hitch + self.L_trailer) + self.desired_trailer_path[0,0]
		x0[1] = np.sin(self.desired_trailer_path[0,3])*(self.ego_to_hitch + self.L_trailer) + self.desired_trailer_path[0,1]
		x0[2] = self.desired_trailer_path[0,3];
		x0[3] = 0;

		print("Initial state of the car is ", x0)
		print("Initial state of the trailer is ", self.desired_trailer_path[0,:])

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
		self.add_initial_state_constraint(x[0], x0);

		# TODO :: Add saturation constraints
		self.add_input_constraints(u, dt);
		self.add_state_constraints(x);

		# TODO :: Add dynamics constraints
		self.add_dynamic_constraints(x,u, dt);

		# TODO :: Add obstacle constraints
		self.add_obstacle_constraints(x);

		# TODO :: Add objective function
		self.add_costs(x, u);
		# self.add_actuation_cost(u);

		#self.add_quadratic_cost_for_car_state(x);
		
		# TODO :: Add warm start
		# self.add_warm_start	(x,u,dt);

		solver = SnoptSolver();
		# solver = IpoptSolver();
		result = solver.Solve(self.prog);
		print("MPC solution complete!")
		x_result = result.GetSolution(x);	
		# print(x_result)
		print(result.get_solution_result())

		trailer_traj = []
		for i in range(self.N):
			x_car = x_result[i,:]
			x_trailer = self.get_trailer_position_from_car_state(x_car);
			trailer_traj += [x_trailer];
		

		self.trailer_traj = np.array(trailer_traj);
		self.car_traj = np.array(x_result[:,:3]);
		self.hitch = self.car_traj[:, :2] - self.ego_to_hitch*np.array([np.cos(self.car_traj[:,2]), np.sin(self.car_traj[:,2])]).T
		for x in self.car_traj:
			print(self.world.is_collided(x))

		#Save car traj as a csv file
		result_file_name = "mpc_wu_chen_2_obs2.csv"
		# np.savetxt("/home/aadith/Desktop/f1_tenth/workspace/src/project/mpc_paths/" + result_file_name, self.car_traj, delimiter=",")
		# np.savetxt("/home/aadith/Desktop/f1_tenth/workspace/src/project/mpc_paths/trailer_" + result_file_name, self.trailer_traj, delimiter=",")

		# Plot the trajectory
		fig, ax = plt.subplots(figsize=(10, 10))

		def animate(i):
			# set the range of x axis and y axis
			ax.set_xlim(-5, 5)
			ax.set_ylim(-5, 5)
			ax.axis('equal')
			ax.plot(self.desired_trailer_path[:,0], self.desired_trailer_path[:,1], "b--o", markersize=2);
			ax.plot(self.trailer_traj[:,0], self.trailer_traj[:,1], "r--o", markersize=2);
			ax.plot(self.car_traj[i,0], self.car_traj[i,1],"cx", markersize=10);
			# Plot a the hitch
			ax.plot([self.car_traj[i, 0], self.hitch[i, 0], self.trailer_traj[i, 0]],
					[self.car_traj[i, 1], self.hitch[i, 1], self.trailer_traj[i, 1]],
					'--o', color="magenta", markersize=3)
			# Plot obstacles
			ax.plot(self.occupied_pixels[:,0], self.occupied_pixels[:, 1], '.', color="black", markersize=4)

			ax.set_title("Single Trailer Trajectory Generation")
			ax.set_xlabel("X (m)")
			ax.set_ylabel("Y (m)")
			ax.legend(['Desired Trailer Trajectory', 'Trailer Trajectory', 'Car Trajectory', 'Hitch Connection', 'Obstacles'])

		# Call the animation
		ani = FuncAnimation(fig, animate, frames=self.N, interval=200, repeat=False)

		# Show the plot
		plt.show()








################### Testing #######################

file = "../maps/wu_chen_map2_obs2"
# file = "/home/aadith/Desktop/f1_tenth/workspace/src/project/maps/wu_chen_map1"
car = Car(file);
car.compute_mpc_feedback();
# pdb.set_trace()

# car.world.is_collided([-1.94,-0.48]);
# car.world.is_collided([-1.85, -0.5]);
# car.world.is_collided([0, 0.1]);
# # car.world.is_collided([1.99,-1.78]);
# car.world.is_collided([1.96,-1.78]);
# car.world.is_collided([-1.6,0.206]);
# car.world.is_collided([-0.845,0.654]);


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