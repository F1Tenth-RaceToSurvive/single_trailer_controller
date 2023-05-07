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

from map import *

class Car(object):
	def __init__(self, file = "/home/aadith/Desktop/f1_tenth/workspace/src/project/maps/wu_chen_map1"):

		self.nx = 4 # xc, yc , yawc, hitch
		self.nu = 2 # v, steering angle
		self.dt = 0.1 # time step
		self.N = 10 # horizon length

		self.Q = np.diag([1,1,1,1]);
		self.R = np.diag([0,1]);

		self.prog = MathematicalProgram();
		# TODO :: Fill values
		self.L_car = -1;
		self.L_trailer = -1;
		self.ego_to_hitch = -1;
		steering_angle_limit = 22;  # In degrees
		self.hitch_limit = 60;  # In degrees

		self.umax = np.array([4, steering_angle_limit*math.pi/180]);
		self.umin = np.array([0, -steering_angle_limit*math.pi/180]);
	
		self.world = Map(file);

	# Write the continuous dynamics of the car system with single trailer here - on axle trailer from common road vehicles source
	def continuous_time_full_dynamics(self, x, u):

		# xc, yc, yawc, hitch
		xdot = np.zeros((self.nx,1))
		xdot[0] = u[0] *np.cos(x[2])
		xdot[1] = u[0]*np.sin(x[2])
		
		
		# TODO :: Fill values -> check if correct it happens to go in the opposite direction<?>
		xdot[2] = u[0]*np.tan(u[1])/self.L_car
		xdot[3] = -u[0]*(np.sin(x[3])/self.L_trailer + np.tan(u[1])/self.hitch_to_rear)
		return xdot
	
	def take_dynamics_step(self,x,u):
		f = self.continuous_time_full_dynamics(x,u);
		x_new = np.zeros(x.shape);
		x_new[0] = x[0] + f[0]*self.dt;
		x_new[1] = x[1] + f[1]*self.dt;
		x_new[2] = x[2] + f[2]*self.dt;
		x_new[3] = x[3] + f[3]*self.dt;

		return x_new;

	def add_initial_state_constraint(self,x,x0):
		self.prog.AddBoundingBoxConstraint(x0,x0,x[0]);

	def add_input_constraints(self, u):
		for ui in u:
			self.prog.AddBoundingBoxConstraint(self.umin[0], self.umax[0], ui[0])
			self.prog.AddBoundingBoxConstraint(self.umin[1], self.umax[1], ui[1])

	def add_state_constraints(self,x):
		for xi in x:
			self.prog.AddBoundingBoxConstraint(-self.hitch_limit * math.pi/180, self.hitch_limit * math.pi/180, xi[3]) # hitch angle limit
			
	def add_dynamic_constraints(self,x,u):
		for i in range(self.N-1):
			x_new = self.take_dynamics_step(x[i],u[i]);
			for j in range(self.nx):
				self.prog.AddConstraint(x_new[j] == x[i+1][j]);
		
	def trailer_cost_evaluator(self, x):
			
			cost = 0;
			for i in range(self.N):
				x_current = x[i][:]
				tx_car = -self.ego_to_hitch - self.trailer_length*np.cos(x_current[3])
				ty_car = -self.trailer_length*np.sin(x_current[3])
				
				T = np.array([[np.cos(x_current[2]), -np.sin(x_current[2]), x_current[0]], 
							[np.sin(x_current[2]), np.cos(x_current[2]), x_current[1]], 
							[0, 0, 1]]);

				t = np.array([[tx_car], [ty_car], [1]]);
				t_w = np.matmul(T, t);
				t_w = t_w.flatten()[:2];

				cost += np.linalg.norm(t_w  - desired_trailer[i])**2;
			return cost

	def add_tracking_cost(self, x, target_pos):
		def collisionCostHelper(vars):
			car_xy = vars.reshape((-1, self.N))
		
			return self.trailer_cost_evaluator(x)


		vars = np.zeros((self.N*self.nx,), dtype = 'object')
		vars = x.flatten()
		# vars[:self.N] = (x[:][0]).reshape((-1,))		#First N coordinates are x coordinates
		# vars[self.N:] = (x[:][1]).reshape((-1,))		#Next N coordinates are y coordinates
		self.prog.AddCost(collisionCostHelper, vars);

	def add_obstacle_constraints(self, x):
		def obsHelper(vars):
			x = vars.reshape((-1, self.N))
			return self.check_collisions(x)
		
		vars = np.zeros((self.N*self.nx,), dtype = 'object')
		vars = x.flatten()
		# vars[:self.N] = (x[:][0]).reshape((-1,))		#First N coordinates are x coordinates
		# vars[self.N:] = (x[:][1]).reshape((-1,))		#Next N coordinates are y coordinates
		lb = np.zeros((self.N,), dtype = 'object')
		self.prog.AddConstraint(obsHelper,lb, lb, vars)

	def check_collisions(self, x):
		collision_states = np.zeros((self.N,))
		for i,xi in enumerate(x):
			if self.world.is_collided(xi):		#Takes times-step i, and state i and checks if has collided or not				return;
				collision_states[i] = 1;
		return collision_states

	def add_actuation_cost(self, u):
		for ui in u:
			self.prog.AddQuadraticCost(ui @ self.R@ ui.T);

	def compute_mpc_feedback(self):
			prog = MathematicalProgram()
			
			#Initialize the decision variables
			x = np.zeros((self.N, self.nx), dtype="object")
			for i in range(self.N):
				x[i] = prog.NewContinuousVariables(self.nx, "x_" + str(i))
			u = np.zeros((self.N-1, self.nu), dtype="object")
			for i in range(self.N-1):
				u[i] = prog.NewContinuousVariables(self.nu, "u_" + str(i))

			# TODO :: Add Initial state constraint

			# TODO :: Add saturation constraints
			self.add_input_constraints(u);
			self.add_state_constraints(x);
			# TODO :: Add dynamics constraints
			self.add_dynamic_constraints(x,u);

			# TODO :: Add obstacle constraints
			self.add_obstacle_constraints(x);

			# TODO :: Add objective function
			self.add_tracking_cost(x, target_pos);
			self.add_actuation_cost(u);

			solver = SnoptSolver();
			result = solver.Solve(prog);
			u_mpc = result.GetSolution(u)[0];	













################### Testing #######################

car = Car();

# x0 = np.array([0,0,0,0]);
# u0 = np.array([1,0.1]);

# traj = [];
# for i in range(0,100):
# 	traj += [x0];
# 	x0 = car.take_dynamics_step(x0,u0,0.1);
# 	print(x0);

# #plot the trajectory
# traj = np.array(traj);
# plt.plot(traj[:,0],traj[:,1]);
# plt.show();