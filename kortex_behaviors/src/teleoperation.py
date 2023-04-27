#!/usr/bin/env python

import rospy
import sys, tty, termios
from kortex_driver.srv import *
from kortex_driver.msg import *
import time

execute_action_full_name = '/' + 'my_gen3_lite' + '/base/execute_action'
rospy.wait_for_service(execute_action_full_name)
execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

def getch():
    old_settings = termios.tcgetattr(0)
    new_settings = old_settings[:]
    new_settings[3] &= ~termios.ICANON
    try:
        termios.tcsetattr(0, termios.TCSANOW, new_settings)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(0, termios.TCSANOW, old_settings)
    return ch

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
	last_action_notif_type = None
	req = ExecuteActionRequest()
	trajectory = WaypointList()

	trajectory.waypoints.append(FillCartesianWaypoint(x, y, z, t_x, t_y, t_z, 0))

	trajectory.duration = 0
	trajectory.use_optimal_blending = False

	req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

	# Call the service
	execute_action(req)
	# try:
	# except rospy.ServiceException:
	# 	rospy.logerr("Failed to call ExecuteWaypointTrajectory")
	# 	return False
	# else:
	# 	while not rospy.is_shutdown():
	# 		if (last_action_notif_type == ActionEvent.ACTION_END):
	# 			rospy.loginfo("Received ACTION_END notification")
	# 			return True
	# 		elif (last_action_notif_type == ActionEvent.ACTION_ABORT):
	# 			rospy.loginfo("Received ACTION_ABORT notification")
	# 			return False
	# 		else:
	# 			time.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('teleoperation')
    cart_pub = rospy.Publisher('/my_gen3_lite/in/cartesian_velocity', TwistCommand, queue_size=10)

    x, y, z, t_x, t_y, t_z = 0.24,  -0.073, 0.118, 10.767, 177.873, 82.773

    print('[INFO]. Moving to the initial position')
    
    home = [x, y, z, t_x, t_y, t_z]

    rospy.loginfo('Moving to Home position.')
    move_cartesian_point(x, y, z, t_x, t_y, t_z)
    comm = None
    coord_delta, rpy_delta = 0.01, 0.01

    while comm is not '`':
        if comm == 'd':
            x += coord_delta
        elif comm == 'a':
            x -= coord_delta
        elif comm == 'w':
            y += coord_delta
        elif comm == 's':
            y -= coord_delta
        elif comm == 'e':
            z += coord_delta
        elif comm == 'q':
            z -= coord_delta
        elif comm == 'l':
            t_x += rpy_delta
        elif comm == 'j':
            t_x -= rpy_delta
        elif comm == 'i':
            t_y += rpy_delta
        elif comm == 'k':
            t_y -= rpy_delta
        elif comm == 'o':
            t_z += rpy_delta
        elif comm == 'u':
            t_z -= rpy_delta
        elif comm == ' ':
            break
        
        move_cartesian_point(x, y, z, t_x, t_y, t_z)
        comm = getch()
        print('[POSE]. X : {}, Y: {}, Z: {}, t_x : {}, t_y: {}, t_z: {}'.format(x, y, z, t_x, t_y, t_z))
        print('[INFO]. Command : {}'.format(comm))
    
    print('[INFO]. Moving to the initial position')
    move_cartesian_point(home[0], home[1], home[2], home[3], home[4], home[5])
    print('[INFO]. Exiting the Teleoperation')
