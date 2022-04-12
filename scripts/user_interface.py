#! /usr/bin/env python

"""! 
 
@file user_interface.py
<div><b>ROS Node Name</b> 
     <ul><li>user_interface</li></ul></div>
@brief a very simple user interface from shell
 
@authors Carmine Tommaso Recchiuto
@version v1.0

<b>Description:</b> <br>
<p>
When the simulation is started, the console asks to press '1' to start 
the motion of the robot, and to press '0' for stopping the motion. This is
the interface this node provides to the user. The node interacts with the
node @ref state_machine.cpp : when the user sends '1', the command "start"
is sent to the state_machine.cpp node. "stop" is sent when the user presses
any other key different from '1'. <br><br>
There's no error handling. Just a simple implementation. 
</p>

<b>UML component</b><br>
(See ... the overal architecture, for further informations)<br>
<img src="" alt="TODO uml"/><br>

<b>Clients:</b> <br>
<ul>
    <li>
			<i>/user_interface</i> : Command.srv <br>
			see the service \ref state_machine.cpp <br><br>
		</li>
</ul>

<b>TODOs</b><br>
@todo review the logs from this node

"""

import rospy
import time
from rt2_assignment1.srv import Command

def main():
	rospy.init_node('user_interface')
	ui_client = rospy.ServiceProxy('/user_interface', Command)
	time.sleep(10)
	rate = rospy.Rate(20)
	x = int(input("\nPress 1 to start the robot "))
	while not rospy.is_shutdown():
		if (x == 1):
			ui_client("start")
			x = int(input("\nPress 0 to stop the robot "))
		else:
			print("Please wait, the robot is going to stop when the position will be reached")
			ui_client("stop")
			x = int(input("\nPress 1 to start the robot "))
			
if __name__ == '__main__':
	main()
