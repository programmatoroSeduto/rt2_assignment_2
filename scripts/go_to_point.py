#! /usr/bin/env python

"""! 
 
@file go_to_point.py
<div><b>ROS Node Name</b> 
     <ul><li>go_to_point</li></ul></div>
@brief Implementation of the movement action. 
 
@authors Carmine Tommaso Recchiuto, Francesco Ganci (S4143910)
@version v1.0

<b>Description:</b> <br>
<p>
This node moves the robot towards a given target (position+orientation) 
using a simple state machine. It is implemented as a ROS action. 
</p>

<b>UML component</b><br>
(See ... the overal architecture, for further informations)<br>
<img src="" alt="TODO uml"/><br>

<b>Publishers:</b> <br>
<ul>
    <li>
        <i>/cmd_vel</i> : Twist <br>
        The node can send a twist to the simulator through this topic. <br><br>
    </li>
</ul>

<b>Subscribers:</b> <br>
<ul>
    <li>
        <i>/odom</i> : Odometry <br><br>
    </li>
</ul>

<b>Providing actions</b> <br>
<ul>
    <li>
            <i>go_to_point</i> : GoToPoint.action <br>
            Move the robot towards a given target pose. The node manages
            a planar pose. 
        </li>
</ul>

<b>TODOs</b><br>
@todo put more comments and tidy up the code

"""

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment_2.srv import Position
import math
import actionlib
from rt2_assignment_2.msg import GoToPointAction, GoToPointGoal, GoToPointResult, GoToPointFeedback

# robot state variables
position_ = Point( )
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None
current_tw_pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

def publish_cmd( tw ):
    '''publish the command both to the simulator and to the GUI.'''
    
    global current_tw_pub_, pub_
    
    pub_.publish( tw )
    current_tw_pub_.publish( tw )

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    # print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    # rospy.loginfo(err_yaw)
    twist_msg = Twist( )
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    # pub_.publish( twist_msg )
    publish_cmd( twist_msg ) 
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    # rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist( )
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        # pub_.publish( twist_msg )
        publish_cmd( twist_msg )
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    # rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    # pub_.publish( twist_msg )
    publish_cmd( twist_msg )
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)

        
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    # pub_.publish( twist_msg )
    publish_cmd( twist_msg )



class GoToPointActionCLass:
    def __init__( self ):
        self.as_ = actionlib.SimpleActionServer( "go_to_point", GoToPointAction, execute_cb=self.goToPoint, auto_start=False )
        self.fb = GoToPointFeedback( )
        self.as_.start( )
        #rospy.loginfo( "go_to_point action started" )
    
    
    def goToPoint( self, goal ):
        # get the desired pose
        desired_position = Point( )
        desired_position.x = goal.x
        desired_position.y = goal.y
        des_yaw = goal.theta
        rospy.loginfo( "[go_to_point] goal: (%f, %f, %f)", goal.x, goal.y, goal.theta )
        
        success = True
        change_state(0)
        
        time_before = rospy.Time.now().to_nsec()
        time_now = -1
        self.fb.progressTime = 0
        self.fb.status = 0
        
        # it could not work because of a slow reading to the topic /odom
        max_distance = math.sqrt( ( position_.x - desired_position.x )**2 + ( position_.y - desired_position.y )**2 )
        
        while True:
            # check for any incoming cancellation request
            if self.as_.is_preempt_requested( ):
                rospy.loginfo( "CANCELLATION REQUEST RECEIVED." )
                self.as_.set_preempted( )
                success = False
                done( )
                change_state( 3 )
                break;
            
            # state machine
            if state_ == 0:
                fix_yaw(desired_position)
            elif state_ == 1:
                go_straight_ahead(desired_position)
            elif state_ == 2:
                fix_final_yaw(des_yaw)
            elif state_ == 3:
                done()
                break
            
            time_now = rospy.Time.now().to_nsec()
            
            # send the feedback
            self.fb.status = state_
            self.fb.actualX = position_.x
            self.fb.actualY = position_.y
            self.fb.actualTheta = yaw_
            self.fb.distance = math.sqrt( ( position_.x - desired_position.x )**2 + ( position_.y - desired_position.y )**2 )
            self.fb.progress = 1 - self.fb.distance / max_distance
            self.fb.progressTime = self.fb.progressTime + (time_now - time_before)
            time_prev = time_now
            self.as_.publish_feedback( self.fb )
            # rospy.loginfo( "[go_to_point] searching ... time: %f, progress: %f%%", self.fb.progressTime, self.fb.progress*100 )
        
        if success:
            # the target has been reached
            # publish the result
            res = GoToPointResult( )
            res.ok = True
            self.as_.set_succeeded( res )
            rospy.loginfo( "[go_to_point] SUCCESS in %f ms", self.fb.progressTime )


def main():
    global pub_, current_tw_pub_
    
    rospy.init_node('go_to_point')
    
    pub_ = rospy.Publisher( '/cmd_vel', Twist, queue_size=1 )
    sub_odom = rospy.Subscriber( '/odom', Odometry, clbk_odom )
    
    # publisher for the actual command
    current_tw_pub_ = rospy.Publisher( '/current_cmd_vel', Twist, queue_size=1 )
    
    act = GoToPointActionCLass( )
    rospy.loginfo( "go_to_point ONLINE" )
    rospy.spin()


if __name__ == '__main__':
    main()
