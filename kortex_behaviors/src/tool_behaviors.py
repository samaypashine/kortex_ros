#!/usr/bin/env python

# /**
#  * @file tool_behaviors.py
#  * @author Samay
#  * @version 1.0
#  * @date 2023-31-03
#  * @copyright Copyright (c) 2023
#  * 
#  */

import os
import termios
import fcntl
import sys
import shutil
import time
import rospy
import math
from kortex_driver.srv import *
from kortex_driver.msg import *
from joint_recorder.msg import recorderMsg
from joint_recorder.srv import *

rospy.init_node('kinova_behaviors')

def cb_action_topic(notif):
	last_action_notif_type = notif.action_event

# Get node params
try:
	HOME_ACTION_IDENTIFIER = 2

	robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
	degrees_of_freedom = rospy.get_param("/" + robot_name + "/degrees_of_freedom", 7)
	is_gripper_present = rospy.get_param("/" + robot_name + "/is_gripper_present", False)

	rospy.loginfo("Using robot_name " + robot_name + " , robot has " + str(degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(is_gripper_present))

	# Init the action topic subscriber
	action_topic_sub = rospy.Subscriber("/" + robot_name + "/action_topic", ActionNotification, cb_action_topic)
	last_action_notif_type = None

	# Init the services
	clear_faults_full_name = '/' + robot_name + '/base/clear_faults'
	rospy.wait_for_service(clear_faults_full_name)
	clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

	read_action_full_name = '/' + robot_name + '/base/read_action'
	rospy.wait_for_service(read_action_full_name)
	read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

	execute_action_full_name = '/' + robot_name + '/base/execute_action'
	rospy.wait_for_service(execute_action_full_name)
	execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

	set_cartesian_reference_frame_full_name = '/' + robot_name + '/control_config/set_cartesian_reference_frame'
	rospy.wait_for_service(set_cartesian_reference_frame_full_name)
	set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

	send_gripper_command_full_name = '/' + robot_name + '/base/send_gripper_command'
	rospy.wait_for_service(send_gripper_command_full_name)
	send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

	activate_publishing_of_action_notification_full_name = '/' + robot_name + '/base/activate_publishing_of_action_topic'
	rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
	activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

	get_product_configuration_full_name = '/' + robot_name + '/base/get_product_configuration'
	rospy.wait_for_service(get_product_configuration_full_name)
	get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

	validate_waypoint_list_full_name = '/' + robot_name + '/base/validate_waypoint_list'
	rospy.wait_for_service(validate_waypoint_list_full_name)
	validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
except:
	is_init_success = False
else:
	is_init_success = True

client_obj = rospy.ServiceProxy('data_recording_service', recorderSrv)
recorder_msg = recorderSrvRequest()

def getch():
	fd = sys.stdin.fileno()
	oldterm = termios.tcgetattr(fd)
	newattr = termios.tcgetattr(fd)
	newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
	termios.tcsetattr(fd, termios.TCSANOW, newattr)

	oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
	fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

	try:        
		while 1:            
			try:
				c = sys.stdin.read(1)
				break
			except IOError: pass
	finally:
		termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
		fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
	return c

def delete_trial(path):
	path = path.split('/')
	path.pop()
	path.pop()
	path = '/'.join(path)

	rospy.logerr('Do you want to delete the data in {}? (Y / N) : '.format(path))
	ans = raw_input("Enter your choice (Y / N) : ")
	if ans == 'Y' or ans == 'y':
		shutil.rmtree(path)
		rospy.loginfo('Deleted the files in {}'.format(path))
	else:
		rospy.logerr('Not deleting the potentially faulty trial.')

def check_data(path):
	data_issue_flag = False

	if len(os.listdir(path)) < 3:
		data_issue_flag = True
	
	if len(os.listdir(path + 'camera_rgb_image/')) <= 10:
		data_issue_flag = True
	
	if len(os.listdir(path + 'touch_image/')) <= 10:
		data_issue_flag = True
	
	if len(os.listdir(path + 'camera_depth_image/')) <= 10:
		data_issue_flag = True
	
	if len(os.listdir(path + 'joint_states/')) < 1:
		data_issue_flag = True
	
	if len(os.listdir(path + 'audio')) == 0:
		data_issue_flag = True
	elif os.path.getsize(path + 'audio/audio.wav') / 1024 < 1:
		data_issue_flag = True
	elif os.path.getsize(path + 'audio/audio.wav') / 1024 > 200 and 'look' in path:
		data_issue_flag = True
	elif os.path.getsize(path + 'audio/audio.wav') / 1024 > 15360 and 'slow' in path:
		data_issue_flag = True
	elif os.path.getsize(path + 'audio/audio.wav') / 1024 > 7168 and 'fast' in path:
		data_issue_flag = True
	elif os.path.getsize(path + 'audio/audio.wav') / 1024 > 18432 and 'twist' in path:
		data_issue_flag = True
	elif os.path.getsize(path + 'audio/audio.wav') / 1024 > 18432 and 'whisk' in path:
		data_issue_flag = True
	elif os.path.getsize(path + 'audio/audio.wav') / 1024 > 8192 and 'poke' in path:
		data_issue_flag = True
	
	if data_issue_flag:
		delete_trial(path)

	return data_issue_flag

def start_recording(path):
	os.system("mkdir -p " + path + "camera_rgb_image/")
	os.system("mkdir -p " + path + "camera_depth_image/")
	os.system("mkdir -p " + path + "touch_image/")
	os.system("mkdir -p " + path + "joint_states/")
	os.system("mkdir -p " + path + "audio/")
	rospy.loginfo("Folder structure created.")

	rospy.loginfo("Setting up the filenames")         
	recorder_msg.command.data = 'set_file_name' 
	recorder_msg.fileName.data = path+"joint_states/joint_states.csv" 
	recorder_msg.topic.data = '/my_gen3_lite/base_feedback/joint_state' 
	client_obj.call(recorder_msg)
	
	recorder_msg.command.data = 'set_file_name' 
	recorder_msg.fileName.data = path+"camera_rgb_image/" 
	recorder_msg.topic.data = 'color_frame_capture' 
	client_obj.call(recorder_msg)
        
	recorder_msg.command.data = 'set_file_name' 
	recorder_msg.fileName.data = path+"camera_depth_image/" 
	recorder_msg.topic.data = 'depth_frame_capture' 
	client_obj.call(recorder_msg)
        
	recorder_msg.command.data = 'set_file_name' 
	recorder_msg.fileName.data = path+"touch_image/" 
	recorder_msg.topic.data = 'touch_frame_capture' 
	client_obj.call(recorder_msg)
        
	recorder_msg.command.data = 'set_file_name' 
	recorder_msg.fileName.data = path+"audio/audio.wav" 
	recorder_msg.topic.data = 'audio_capture'
	client_obj.call(recorder_msg)
	
	rospy.loginfo("Starting Recording")
	recorder_msg.command.data = 'start' 
	client_obj.call(recorder_msg)

def stop_recording():
	rospy.loginfo("Stopping Recording")
	recorder_msg.command.data = 'stop'
	client_obj.call(recorder_msg)

def clean_faults():
	try:
		clear_faults()
	except rospy.ServiceException:
		rospy.logerr("Failed to call ClearFaults")
		return False
	else:
		rospy.loginfo("Cleared the faults successfully")
		rospy.sleep(2.5)
		return True

def open_gripper():
	req = SendGripperCommandRequest()
	finger = Finger()
	finger.finger_identifier = 0
	finger.value = 0.0
	req.input.gripper.finger.append(finger)
	req.input.mode = GripperMode.GRIPPER_POSITION

	rospy.loginfo("Sending the gripper command...")

	# Call the service 
	while True:
		try:
			send_gripper_command(req)
		except rospy.ServiceException:
			rospy.logerr("Failed to call SendGripperCommand")
			return False
		else:
			time.sleep(2.5)
			return True

def close_gripper():
	req = SendGripperCommandRequest()
	finger = Finger()
	finger.finger_identifier = 0
	finger.value = 1.0
	req.input.gripper.finger.append(finger)
	req.input.mode = GripperMode.GRIPPER_POSITION

	rospy.loginfo("Sending the gripper command...")

	# Call the service 
	while True:
		try:
			send_gripper_command(req)
		except rospy.ServiceException:
			rospy.logerr("Failed to call SendGripperCommand")
			return False
		else:
			time.sleep(2.5)
			return True

def FillCartesianWaypoint(new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
	waypoint = Waypoint()
	cartesianWaypoint = CartesianWaypoint()

	cartesianWaypoint.pose.x = new_x
	cartesianWaypoint.pose.y = new_y
	cartesianWaypoint.pose.z = new_z
	cartesianWaypoint.pose.theta_x = new_theta_x
	cartesianWaypoint.pose.theta_y = new_theta_y
	cartesianWaypoint.pose.theta_z = new_theta_z
	cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
	cartesianWaypoint.blending_radius = blending_radius
	waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

	return waypoint

def move_cartesian_point(x, y, z, t_x, t_y, t_z):
	# last_action_notif_type = None
	req = ExecuteActionRequest()
	trajectory = WaypointList()

	trajectory.waypoints.append(FillCartesianWaypoint(x, y, z, t_x, t_y, t_z, 0))

	trajectory.duration = 0
	trajectory.use_optimal_blending = False

	req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
	execute_action(req)

def get_circular_points(x, y, z, t_x, t_y, t_z, radius, num_points):
	angle_per_point = 360 // num_points
	points = [[x, y, z, t_x, t_y, t_z]]

	for i in range(num_points):
		Y = y + radius*math.sin((i * angle_per_point * 3.14) / 180)
		X = x + radius*math.cos((i * angle_per_point * 3.14) / 180)

		points.append([X, Y, z, t_x, t_y, t_z])
	return points

def look_behavior(path, delay):
	start_recording(path)
	rospy.sleep(delay)
	stop_recording()

def stirring_behavior_1(path, tool_x, tool_y, tool_z, t_x, t_y, t_z, radius, rotations, num_points=10, timeDelay=0.5):
	points = get_circular_points(tool_x, tool_y, tool_z, t_x, t_y, t_z, radius, num_points)
	start_recording(path)
	while rotations > 0:
		rospy.loginfo('Rotation Left : {}'.format(rotations))
		for i in range(len(points)-1):
			move_cartesian_point(points[i+1][0], points[i+1][1], points[i+1][2], points[i+1][3], points[i+1][4], points[i+1][5])
			rospy.sleep(timeDelay)
		rotations -= 1
	stop_recording()
	move_cartesian_point(points[0][0], points[0][1], points[0][2], points[0][3], points[0][4], points[0][5])

def stirring_behavior_2(path, tool_x, tool_y, tool_z, t_x, t_y, t_z, radius, twist_angle, rotations, num_points=10, timeDelay=0.5):
	points = get_circular_points(tool_x, tool_y, tool_z, t_x, t_y, t_z, radius, num_points)
	start_recording(path)
	while rotations > 0:
		rospy.loginfo('Rotation Left : {}'.format(rotations))
		for i in range(len(points)-1):
			if i % 2 == 0:
				move_cartesian_point(points[i+1][0], points[i+1][1], points[i+1][2], points[i+1][3], points[i+1][4], points[i+1][5] - twist_angle) ##################### Add the value to theta_xyz
			else:
				move_cartesian_point(points[i+1][0], points[i+1][1], points[i+1][2], points[i+1][3], points[i+1][4], points[i+1][5] + twist_angle) ##################### Add the value to theta_xyz
			rospy.sleep(timeDelay)
		rotations -= 1
	stop_recording()
	move_cartesian_point(points[0][0], points[0][1], points[0][2], points[0][3], points[0][4], points[0][5])

def stirring_behavior_3(path, tool_x, tool_y, tool_z, roll, pitch, yaw, radius, rotations, num_points=10, timeDelay=0.5):
	points = get_circular_points(tool_x, tool_y, tool_z, roll, pitch, yaw, radius, num_points)
	start_recording(path)
	while rotations > 0:
		rospy.loginfo('Rotation Left : {}'.format(rotations))
		for i in range(1, len(points)):
			move_cartesian_point(points[i][0], points[i][1], points[i][2], points[i][3], points[i][4], points[i][5])
			rospy.sleep(timeDelay)
			j = (i + num_points/2) % num_points
			if j == 0:
				move_cartesian_point(points[num_points][0], points[num_points][1], points[num_points][2], points[num_points][3], points[num_points][4], points[num_points][5])
			else:
				move_cartesian_point(points[j][0], points[j][1], points[j][2], points[j][3], points[j][4], points[j][5])
			rospy.sleep(timeDelay)
		rotations -= 1
	stop_recording()
	move_cartesian_point(points[0][0], points[0][1], points[0][2], points[0][3], points[0][4], points[0][5])

def stirring_behavior_4(path, tool_x, tool_y, tool_z, roll, pitch, yaw, radius, rotations, num_points=10, timeDelay=0.5):
	points = get_circular_points(tool_x, tool_y, tool_z, roll, pitch, yaw, radius, num_points)
	start_recording(path)
	while rotations > 0:
		rospy.loginfo('Rotation Left : {}'.format(rotations))
		center_ja = [tool_x, tool_y, tool_z, roll, pitch, yaw]
		for i in range(1, len(points)):
			move_cartesian_point(points[i][0], points[i][1], points[i][2] - 0.01, points[i][3], points[i][4], points[i][5])
			rospy.sleep(timeDelay)
			move_cartesian_point(center_ja[0], center_ja[1], center_ja[2] + 0.05, center_ja[3], center_ja[4], center_ja[5])
			rospy.sleep(timeDelay)
		rotations -= 1
	stop_recording()
	move_cartesian_point(points[0][0], points[0][1], points[0][2], points[0][3], points[0][4], points[0][5])

if __name__ == '__main__':

	clean_faults()
	open_gripper()
	sensor_data_path = "/home/samay/Desktop/Tool_Dataset/"
	robot_name = 'gen3-lite'

	tools = ['plastic-spoon', 'wooden-chopstick', 'metal-scissor', 'metal-whisk', 'plastic-knife', 'wooden-fork']
	contents = ["chickpea", "wheat", "split-green-pea", "kidney-bean", "chia-seed",
				"salt", "cane-sugar", "empty", "plastic-bead",
				"styrofoam-bead", "wooden-button", "glass-bead", "metal-nut-bolt", "water", "detergent"]

	tool = tools[int(input('[INTERACTIVE]. Select one of the following tools:\n1. {}\n2. {}\n3. {}\n4. {}\n5. {}\n6. {}\n* Choice : '.format(tools[0], tools[1], tools[2], tools[3], tools[4], tools[5])))-1]
	content = contents[int(input('[INTERACTIVE]. Select one of the following contents:\n1. {}\n2. {}\n3. {}\n4. {}\n5. {}\n6. {}\n7. {}\n8. {}\n9. {}\n10. {}\n11. {}\n12. {}\n13. {}\n14. {}\n15. {}\n* Choice : '.format(contents[0], contents[1], contents[2], contents[3], contents[4], contents[5], contents[6], contents[7],contents[8], contents[9], contents[10], contents[11], contents[12], contents[13], contents[14])))-1]
	rospy.loginfo('Selcted Tool 	: {}'.format(tool))
	rospy.loginfo('Selcted Object 	: {}'.format(content))

	sensor_data_path += robot_name + '_' + tool + '/' + content + '/'
        
	os.system('mkdir -p {}'.format(sensor_data_path))

	trial_num = len(os.listdir(sensor_data_path))

	sensor_data_path += 'trial-{}'.format(trial_num) + '_' + str(time.strftime("%d-%m-%Y-%H-%M-%S", time.gmtime())) + '/'

	tools_x = {'wooden-chopstick': 0.18,
			'plastic-spoon': 0.20,
			'metal-scissor': 0.22,
			'metal-whisk': 0.21,
			'plastic-knife': 0.21,
			'wooden-fork': 0.20}
	
	tools_y = {'wooden-chopstick': -0.035,
			'plastic-spoon': -0.030,
			'metal-scissor': -0.025,
			'metal-whisk': -0.03,
			'plastic-knife': -0.03,
			'wooden-fork': -0.03}
	
	tools_z = {'wooden-chopstick': 0.220,
			'plastic-spoon': 0.140,
			'metal-scissor': 0.110,
			'metal-whisk': 0.110,
			'plastic-knife': 0.175,
			'wooden-fork': 0.210}


	tool_x = tools_x[tool]
	tool_y = tools_y[tool]
	tool_z = tools_z[tool]
	tool_t_x = 10.767
	tool_t_y = 177.873
	tool_t_z = 82.773

	rospy.loginfo('tool_x : {}'.format(tool_x))
	rospy.loginfo('tool_y : {}'.format(tool_y))
	rospy.loginfo('tool_z : {}'.format(tool_z))
	rospy.loginfo('tool_t_x : {}'.format(tool_t_x))
	rospy.loginfo('tool_t_y : {}'.format(tool_t_y))
	rospy.loginfo('tool_t_z : {}'.format(tool_t_z))

	rospy.loginfo('Moving to Home position.')
	move_cartesian_point(tool_x, tool_y, tool_z+0.10, tool_t_x, tool_t_y, tool_t_z)
	rospy.sleep(1.5)
	rospy.loginfo('Attach the Tool & Press any key to continue ....')
	getch()

	radius = 0.012
	data_issue_flag = False

	behavior_name = '1-look'
	rospy.loginfo('Look Behavior')
	look_behavior(sensor_data_path + behavior_name + '/', delay=1.0)
	rospy.sleep(2.0)
	data_issue_flag = check_data(sensor_data_path + behavior_name + '/')
	
	# Close gripper after the look behavior
	close_gripper()

	if not data_issue_flag:
		behavior_name = '2-stirring-slow'
		stirring_behavior_1(sensor_data_path + behavior_name + '/', tool_x, tool_y, tool_z, tool_t_x, tool_t_y, tool_t_z, radius=radius, rotations=5, num_points=20, timeDelay=0.75)
		rospy.loginfo('Stirring Behavior 1')
		rospy.sleep(3.0)
		data_issue_flag = check_data(sensor_data_path + behavior_name + '/')

	if not data_issue_flag:
		behavior_name = '3-stirring-fast'
		rospy.loginfo('Stirring Behavior 2')
		stirring_behavior_1(sensor_data_path + behavior_name + '/', tool_x, tool_y, tool_z, tool_t_x, tool_t_y, tool_t_z, radius=radius, rotations=5, num_points=30, timeDelay=0.2)
		rospy.sleep(3.0)
		data_issue_flag = check_data(sensor_data_path + behavior_name + '/')

	if not data_issue_flag:
		behavior_name = '4-stirring-twist'
		rospy.loginfo('Stirring Behavior 3')
		stirring_behavior_2(sensor_data_path + behavior_name + '/', tool_x, tool_y, tool_z, tool_t_x, tool_t_y, tool_t_z, radius=radius/10, twist_angle=40.0, rotations=1, num_points=30, timeDelay=3.0)
		rospy.sleep(3.0)
		data_issue_flag = check_data(sensor_data_path + behavior_name + '/')

	if not data_issue_flag:
		behavior_name = '5-whisk'
		rospy.loginfo('Stirring Behavior 4')
		stirring_behavior_3(sensor_data_path + behavior_name + '/', tool_x, tool_y, tool_z, tool_t_x, tool_t_y, tool_t_z, radius=radius, rotations=2, num_points=20, timeDelay=1.0)
		rospy.sleep(3.0)
		data_issue_flag = check_data(sensor_data_path + behavior_name + '/')

	if not data_issue_flag:
		behavior_name = '6-poke'
		rospy.loginfo('Stirring Behavior 5')
		stirring_behavior_4(sensor_data_path + behavior_name + '/', tool_x, tool_y, tool_z, tool_t_x, tool_t_y, tool_t_z, radius=radius, rotations=1, num_points=20, timeDelay=0.90)
		rospy.sleep(3.0)
		data_issue_flag = check_data(sensor_data_path + behavior_name + '/')

	if data_issue_flag:
		delete_trial(sensor_data_path + behavior_name + '/')
	
	rospy.loginfo('Moving to Home position.')
	move_cartesian_point(tool_x, tool_y, tool_z+0.10, tool_t_x, tool_t_y, tool_t_z)
	rospy.loginfo('Opening the Gripper')
	open_gripper()
