#! /usr/bin/env python

"""A simple motion planning algorithm in obstacle-free space.

This is the implementation of a simple motion planning algorithm
as a state machine. Working only in a obstacle-free space: the 
implicit assumption is that there is never an obstacle from the
current position to the target one. 

Publishers:
    **/cmd_vel** (geometry_msgs/Twist): 
        command given to the robot
    **/current_cmd_vel** (geometry_msgs/Twist): 
        a copy of the command is sent to jupyter.

Subscribers:
    **/odom** (nav_msgs/Odometry): 
        the current position/orientation/motion of the robot. 

Providing Actions:
    **/go_to_point** (rt2_assignment_2/GoToPoint):
        the motion planning algorithm
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
"""geometry_msgs/Point: current robot position """

yaw_ = 0
"""Float: current orientation of the robot about the vertical axis (z) """

state_ = 0
"""Int: current state of the state machine """

pub_ = None
"""ROS_publisher_handle: publisher to /cmd_vel """

current_tw_pub_ = None
"""ROS_publisher_handle: publisher to /current_cmd_vel """

# parameters for control
yaw_precision_ = math.pi / 9
"""Float: first precision for the orientation

In order to reach the final orientation, the node checks if
the difference between the desider orientation and the target
orientation is under a certain tolerance, i.e. this number. 

+/- 20 degree allowed. 
 """

yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
"""Float: second precision for the orientation

In order to reach the final orientation, the node checks if
the difference between the desider orientation and the target
orientation is under a certain tolerance, i.e. this number. 

+/- 2 degree allowed. More precision compared to the other
precision value. 
 """
 
dist_precision_ = 0.1
"""Float: precision for the distance from a target.

While trying to go at a certain position, the system checks 
regularly if the distance between the current (x,y) position
and the target one is below a certain threshold, that is this 
number. 
"""

kp_a = -3.0 
"""Float: Gain yaw error to z angular velocity

This gain value is applied as proportionality factor between
the yaw distance and the angular velocity to give to the robot
during the orientation phases. 
"""

ub_a = 0.6
"""Float: maximum robot angular velocity. """

lb_a = -0.5
"""Float: minimum angular velocity of the robot. """

ub_d = 0.6
"""Float: maximum linear velocity of the robot. """



def publish_cmd( tw ):
    """publish the command both to the simulator and to the GUI.
    
    The punction simply publishes the same twist on two topics:
    /cmd_vel and /current_cmd_vel. 
    
    Args:
        tw (geometry_msgs/Twist): the twist to publish.
    """
    
    global current_tw_pub_, pub_
    
    pub_.publish( tw )
    current_tw_pub_.publish( tw )



def clbk_odom( msg ):
    """ Read an incoming odometry message.
    
    The function, called when a new Odometry message is published
    from the "simulator", takes the message and stores its fields
    in the variables position_ and yaw_ .
    
    Args:
        msg (geometry_msgs/Odometry): the odometry from the simulation
    """
    
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
    """Update the current status of the state machine.
    
    The function (actually) doesn't do nothing more than the name
    says: simply it sets the state_ variable as the one passed as
    argument. 
    
    Args:
        state (int): the new state of the state machine.
    """
    
    global state_
    state_ = state



def normalize_angle( angle ):
    """Normalize the angle in [-pi, pi].
    
    Args:
        angle (Float): the angle to be normalized
    
    Returns:
        Float: the normalized angle in [-pi, pi]
    """
    
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    
    return angle



def fix_yaw( des_pos ):
    """First step of the algorithm
    
    The function is called once when the state machine is performing
    the initial alignment towards the target to reach. This turns the 
    robor of one step at time. If the current angle is under a certain
    threshold (here the yaw_precision_2_ is employed) the function 
    sends the stop twist, and then changes the status of the machine
    calling the funtion change_state. 
    
    Args:
        des_pos (geometry_msgs/Point):
            where the target is located; its (x, y, z) coordinates.
    """
    
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



def go_straight_ahead( des_pos ):
    """Do a step straight to the target. 
    
    The function performs the movement call by call, so it is called in
    a loop until the robot isn't in the target position (under a certain
    threshold upon the distance).
    
    If the distance is higher than the threshold, the function sends a 
    command that moves the robot straight towards the target. A constant
    head velocity is sent, in this case 0.3 m/s. 
    Otherwise, the function sends the "stop command" (i.e. a zero twist) and 
    changes the status of the state machine. 
    
    Notice that the curve followed by the robotoduring the motion is not 
    properly a "straight" line, because the function also deal with a
    deviation of the head towards the target. If the robot would follow a
    trajectory which is not well oriented to the target, the system could
    move the robot endlessy. So a minimum hcorrection of the head orientation
    is needed. 
    
    Args:
        des_pos (geometry_msgs/Point):
            where the target is located; its (x, y, z) coordinates.
    """
    
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    # rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist( )
        twist_msg.linear.x = 0.3
        
        """
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d
        """
        
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



def fix_final_yaw( des_yaw ):
    """Last step: align the robot with the goal yaw. 
    
    The function changes the orientation of the robot call by
    call. The robot is supposed to be located in the target position,
    hence only the orientation should be changed to achieve the goal. 
    
    If the (normalized) difference between the desired orientation
    and the current one is greater than a certain threshold (here
    yaw_precision_2_ has been applied) the function sends a command
    that turns the robot toeards the goal.
    Otherwise (the difference is under the threshold) the state of 
    the machine is changed. 
    
    Args:
        des_yaw (Float): the target orientation
    """
    
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



def done( ):
    """Mission completed (or aborted)
    
    The function stops the mission: the zero twist is 
    sent to the actuators. The mission is over. 
    """
    
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    # pub_.publish( twist_msg )
    publish_cmd( twist_msg )



class GoToPointActionCLass:
    """Implementation of the motion plannig algorithm as ROS1 action
    
    The motion planning algorithm here is implemented as ROS1 
    Action Server. The class contains the working cycle of the
    node, as well as a onstructor able to correctly instanciate
    the ROS1 action server. 
    """
    
    def __init__( self ):
        """cllass constructor. 
        
        Instanciation of the ROS1 action service. 
        """
        
        self.as_ = actionlib.SimpleActionServer( "go_to_point", GoToPointAction, execute_cb=self.goToPoint, auto_start=False )
        self.fb = GoToPointFeedback( )
        self.as_.start( )
        #rospy.loginfo( "go_to_point action started" )
    
    
    def goToPoint( self, goal ):
        """Task of the action go_to_point
        
        Here's the implementation of the state machine as a 
        "endless" loop. 
        
        Args:
            goal (GoToPointRequest):
                the objective of the mission. 
        """
        
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



def main( ):
    """Main routine of the node.
    
    Instanciation of the channels (/cmd_vel, /current_cmd_vel,
    /odom and the action /go_to_point), then spin. 
    """
    
    global pub_, current_tw_pub_
    
    rospy.init_node('go_to_point')
    
    pub_ = rospy.Publisher( '/cmd_vel', Twist, queue_size=1 )
    sub_odom = rospy.Subscriber( '/odom', Odometry, clbk_odom )
    
    # publisher for the actual command
    current_tw_pub_ = rospy.Publisher( '/current_cmd_vel', Twist, queue_size=1 )
    
    act = GoToPointActionCLass( )
    rospy.loginfo( "go_to_point ONLINE" )
    rospy.spin( )


if __name__ == '__main__':
    main()
