import numpy as np
from math import *
# from rclpy.node import Node
import os
from itertools import *
from topic import *

PATH = './SARSA/'
ALPHA = 0.1
GAMMA = 0.9
EPISODES = 25000

epsilon = 1 
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILON_DECAYING - START_EPSILON_DECAYING)

class SARSA():

	def __init__(
			self, 
			learning_rate = ALPHA,
			discount_factor = GAMMA, 
			episodes = EPISODES, 
			epsilon = epsilon,
			epsilon_decay_value = epsilon_decay_value
			) -> None:
     
		self.ALPHA = learning_rate
		self.GAMMA = discount_factor
		self.episodes = episodes
		self.epsilon = epsilon
		self.epsilon_decay_value = epsilon_decay_value

		self.lidar_data_space = self.create_lidar_states()
		self.goal_distance_space = [0,1,2,3,4,5,6,7,8,9,10,11]
		self.action_space = [0,1,2,3,4,5,6,7]

		

		self.lidar_n = len(self.lidar_data_space)
		self.goal_n = len(self.goal_distance_space)
		self.action_n = len(self.action_space)
		self.QTable = self.create_QTable()
		
		self.run()


  
	def create_lidar_states(self):
		x1 = set((0,1,2))
		x2 = set((0,1,2))
		x3 = set((0,1,2))
		x4 = set((0,1,2))
		x5 = set((0,1,2))
		x6 = set((0,1,2))
		x7 = set((0,1,2))
		x8 = set((0,1,2))
		lidar_state = set(product(x1,x2,x3,x4,x5,x6,x7,x8))
		return np.array(list(lidar_state))
		
	def get_state(self, lidar, goal):
		state = [lidar, goal]
		return tuple(state)

	def create_QTable(self):
		QTable = np.random.uniform(low=-2, high=0, size=([self.lidar_n] + [self.goal_n] + [self.action_n]))
		return QTable

	def choose_action(self, state):
		if np.random.rand() <= epsilon:
			action = np.random.choice(self.action_space)  
		else:
			action = np.argmax(self.QTable[state])
		return action
	
	def update_QTable(self, state, action, reward, next_state, next_action):
		next_q_value = self.QTable[next_state + (next_action,)]
		current_q_value = self.QTable[state + (action,)]
		target_q_value = reward + (self.GAMMA * next_q_value)
		new_q_value = (1 - self.ALPHA) * current_q_value + (self.ALPHA * target_q_value)
		self.QTable[state + (action,)] = new_q_value


	def run(self):
		lidar = [2]*8
		goal = 9
		state = self.get_state(lidar, goal)
		action = self.choose_action(state)

		print(f"State: {state}")
		print(f"Action: {action}")

		next_lidar = [1]*8
		next_goal = 8
		next_state = self.get_state(next_lidar, next_goal)
		next_action = self.choose_action(next_state)
		reward = -1

		print(f"Next State: {next_state}")
		print(f"Next Action: {next_action}")

		self.update_QTable(state, action, reward, next_state, next_action)
		




sarsa = SARSA()





    