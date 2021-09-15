#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ModelStates 
import time
import math


class data_collector:
	uncertainty = "medium"
	def __init__(self):
		rospy.init_node("data_collector", anonymous=True)
		rospy.Subscriber("/mavros/local_position/odom", Odometry, self.drone_odom_cb)
		rospy.Subscriber("/gazebo/model_states",  ModelStates, self.obstacle_state_cb)
		self.drone_pos = [-1000, -1000, -1000]
		self.obstacle_pos = [-1000, -1000, -1000]
		self.ob_idx = 0
		self.first_time = True
		self.start_pos = [-9.0, -4.7, 0.65]
		self.goal_pos = [8.0, 7.3, 0.6]



	def drone_odom_cb(self, odom):
		self.drone_pos[0] = odom.pose.pose.position.x
		self.drone_pos[1] = odom.pose.pose.position.y
		self.drone_pos[2] = odom.pose.pose.position.z

	def obstacle_state_cb(self, all_states):
		if (self.first_time):
			for name in all_states.name:
				if (name == "person_walking_0"):
					break
				self.ob_idx += 1
			self.first_time = False
		obstacle_pose = all_states.pose[self.ob_idx]
		self.obstacle_pos[0] = obstacle_pose.position.x
		self.obstacle_pos[1] = obstacle_pose.position.y
		self.obstacle_pos[2] = obstacle_pose.position.z

	def getDistance(self, pos1, pos2):
		[x1, y1, z1] = pos1
		[x2, y2, z2] = pos2
		return ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5

	def run(self):
		rate = rospy.Rate(10)
		while (self.drone_pos[0] == -1000 or self.obstacle_pos[0] == -1000):
			rospy.loginfo("Waiting for messages...")
			rate.sleep()
		rospy.loginfo("Messages Received!!!")
		experiment_no = "12"
		# path = "/home/zhefan/Desktop/results/DPMPC/uncertainty_" + self.uncertainty
		# path = "/home/zhefan/Desktop/results/deterministic_DPMPC/uncertainty_" + self.uncertainty
		path = "/home/zhefan/Desktop/results/DPMPC_without_TGC/uncertainty_" + self.uncertainty
		# path = "/home/zhefan/Desktop/results/DPMPC/uncertainty_" + self.uncertainty
		f_ob_dis = open(path + "/obstacle_distance/distance" + experiment_no + ".txt", "w")
		f_time = open(path + "/time/time" + experiment_no + ".txt", "w")
		f_length = open(path + "/length/length" + experiment_no + ".txt", "w")
		f_success = open(path + "/success/success" + experiment_no + ".txt", "w")
		
		success_close = False

		start_experiment = False
		end_experiment = False
		delta = 0.2
		while (not start_experiment):
			if (self.getDistance(self.drone_pos, self.start_pos) <= delta):
				start_experiment = True
				start_time = time.time()
				rospy.loginfo("Start experiment!!")
				break
			rate.sleep()

		total_length = 0
		prev_pos = [self.start_pos[0], self.start_pos[1], self.start_pos[2]]
		while (True):

			if (not end_experiment and self.getDistance(self.drone_pos, self.goal_pos) <= 0.1):
				rospy.loginfo("End experiment!!")
				end_time = time.time()
				time_use = str(end_time - start_time)
				rospy.loginfo("Time: " + time_use)
				f_time.write(time_use)
				f_length.write(str(total_length))
				rospy.loginfo("Length: %s", total_length)
				if (not success_close):
					f_success.write(str(1))
				end_experiment = True

			# rospy.loginfo("drone pos: %s, %s, %s", self.drone_pos[0], self.drone_pos[1], self.drone_pos[2])
			# rospy.loginfo("obstacle pos: %s, %s, %s", self.obstacle_pos[0], self.obstacle_pos[1], self.obstacle_pos[2])
			total_length += self.getDistance(self.drone_pos, prev_pos)
			prev_pos = [self.drone_pos[0], self.drone_pos[1], self.drone_pos[2]]
			
			distance = self.getDistance([self.drone_pos[0], self.drone_pos[1], 0], [self.obstacle_pos[0], self.obstacle_pos[1], 0])
			if (distance <= 5):
				f_ob_dis.write(str(distance) + "\n")
				rospy.loginfo("obstacle distance: %s", distance)

			if (not success_close and distance <= 0):
				rospy.loginfo("Failure!!")
				f_success.write(str(0))
				f_success.close()
				success_close = True


			rate.sleep()

if __name__ == "__main__":
	dc = data_collector()
	dc.run()
