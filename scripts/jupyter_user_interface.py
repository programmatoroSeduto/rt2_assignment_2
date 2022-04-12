#! /user/bin/env python

"""! @file jupyter_user_interface.py
<div><b>ROS Node Name</b> 
     <ul><li>jupyter_user_interface</li></ul></div>
@brief This node provides functionalities to support Jupyter in driving the robot. 

@authors Francesco Ganci (S4143910)
@version v1.0

This node implements a support for a user interface built in Jupyter.<br>
Here are the functionalities of the node:<br>
<ul>
<li>The node can switch between authomatic mode and manual mode through a service called '/ui_trigger'</li>
<li>When the authomatic mode is enabled, the random target service is immediately activated</li>
<li>When the manual mode is required through the service, the authomatic behaviour is turned off</li>
</ul>

@see qualcosa

<b>TODOs</b><br>

"""

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from rt2_assignment1.srv import Command, CommandRequest, CommandResponse

### Name for this node
node_name = "jupyter_user_interface"

### Name of the trigger service (being used to set/unset manual mode)
name_ui_trigger = "/ui_trigger"

### service handler (ui_trigger)
srv_ui_trigger = None

### client handle (ui_client)
cli_ui_client = None

### Switch for the manual mode (the init value is also the first mode)
use_manual_mode = True

### service callback for the trigger
def cbk_ui_trigger( trig ):
    """!
    @brief implementation of the trigger service
    
    @param trig (std_srv.srv.SetBoolRequest) if set or not the manual mode
    @returns (std_srv.SetBoolResponse) useless
    """
    global use_manual_mode
    
    rospy.loginfo( "received request from Jupyter : { data:%s }", str( trig.data ) )
    if use_manual_mode and (not trig.data):
        # turn on the authomatic behaviour
        rospy.loginfo( "starting authomatic behaviour ..." )
        cli_ui_client( CommandRequest( command="start" ) )
        rospy.loginfo( "starting authomatic behaviour ... OK!" )
        use_manual_mode = False
        
    elif (not use_manual_mode) and trig.data:
        # turn off the authomatic behaviour
        rospy.loginfo( "stopping authomatic behaviour ..." )
        cli_ui_client( CommandRequest( command="stop" ) )
        rospy.loginfo( "stopping authomatic behaviour ... OK!" )
        use_manual_mode = True
    
    return SetBoolResponse( )

### main function
def main( ):
    """!
    @brief main function of for the node jupyter_user_interface
    """
    rospy.spin( )

if __name__ == "__main__":
    try:
        # init
        rospy.init_node( node_name )
        rospy.loginfo( "init node '%s'", node_name )
        rospy.on_shutdown( lambda : rospy.loginfo( "shutdown node '%s'", node_name ) )
        
        # services and topics
        # --- SERVICE --- /ui_trigger
        rospy.loginfo( "Service '%s' ...", name_ui_trigger )
        srv_ui_trigger = rospy.Service( name_ui_trigger, SetBool, cbk_ui_trigger )
        rospy.loginfo( "Service '%s' ... OK!", name_ui_trigger )
        # --- CLIENT --- /user_interface
        rospy.loginfo( "Client '%s' ...", '/user_interface' )
        rospy.wait_for_service( '/user_interface' )
        cli_ui_client = rospy.ServiceProxy( '/user_interface', Command )
        rospy.loginfo( "Client '%s' ... OK!", '/user_interface' )
        
        # set the first behaviour
        if use_manual_mode:
            rospy.loginfo( "init behaviour: manual mode" )
            cli_ui_client( CommandRequest( command="stop" ) )
        else:
            rospy.loginfo( "init behaviour: authomatic mode" )
            rospy.loginfo( "starting authomatic behaviour ..." )
            cli_ui_client( CommandRequest( command="stop" ) )
            rospy.loginfo( "starting authomatic behaviour ... OK!" )
        
        # launch the node
        main( )
        
    except rospy.ROSException as e:
        rospy.loginfo( "ROS exception occurrend" )
        rospy.loginfo( e.message )