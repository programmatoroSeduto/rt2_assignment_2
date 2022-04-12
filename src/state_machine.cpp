/********************************************//**
 *  
 * \file state_machine.cpp
 * <div><b>ROS Node Name</b> 
 *      <ul><li>state_machine</li></ul></div>
 * \brief Send a command to the robot and manage the movement action. 
 * 
 * \authors Carmine Tommaso Recchiuto, Francesco Ganci (S4143910)
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * Despite of the name, this node is not a state machine: it receives a 
 * command, then calls the action providing a random target. After a 
 * destination is provided to the movement action, the node waits until 
 * the destination is reached. <br><br>
 * The node can also cancel the action: see the description of the 
 * service <i>/user_interface</i>. <br><br>
 * In general the node, when enabled, works in this way. First of all, if
 * there is no target, a new target is obtained from the service 
 * <i>/user_interface</i>; then, the node sends the request to the movement 
 * action. After sent the request, the node executes a check upon the status
 * of the movement action every <i>at least</i> 0.1 seconds (see 
 * \ref WAITING_DELAY) for maximum 30 seconds (see \ref MAX_WAITING_TIME).
 * If the action requires too much time, the request is cancelled. <br><br>
 * Note that the bounds of the target are provided once in the main function:
 * x in [-5.0, 5.0], y in [-5.0, 5.0]. 
 * </p>
 * 
 * <b>UML component</b><br>
 * (See ... the overal architecture, for further informations)<br>
 * <img src="" alt="TODO uml"/><br>
 * 
 * <b>Services:</b> <br>
 * <ul>
 *     <li>
 *          <i>/user_interface</i> : Command.srv <br>
 *          The service can enable or disable the node, and also cancel 
 *          any request sent to the movement action. Send <code>start</code>
 *          to start the movement. Send any other strings to stop the 
 *          working cycle and possibly cancel any active movement action. 
 *      </li>
 * </ul>
 * 
 * <b>Clients:</b> <br>
 * <ul>
 *     <li>
 *          <i>/position_server</i> : RandomPosition.srv <br>
 *          See \ref position_service.cpp
 *      </li>
 * </ul>
 * 
 * <b>Using actions</b> <br>
 * <ul>
 *     <li>
 *          <i>go_to_point</i> : GoToPoint.action <br>
 *          See the py node \ref go_to_point.py
 *      </li>
 * </ul>
 * 
 * <b>TODOs</b><br>
 * \todo UML diagram for this node
 * \todo the command should be case-insensitive
 * \todo review the output to the screen
 * 
 ***********************************************/

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/GoToPointAction.h"
#include "rt2_assignment1/JupyterTargetInfo.h"

#include <string>



/// The minimom delay between one cycle and the next one
#define WAITING_DELAY 0.1



/// The maximum waiting time before the forced break of the cycle
#define MAX_WAITING_TIME 30



/// when true, the node is enabled. 
bool start = false;



/// Shared pointer to the movement action handler
actionlib::SimpleActionClient<rt2_assignment1::GoToPointAction>* acglobal;


/// shared pointer to the info topic towards Jupyter
ros::Publisher* global_jupy;



/********************************************//**
 *  
 * @brief implementation of service <b>/user_interface</b>
 * 
 * @param request The command to execute. If the command is the string "start"
 *          then the node is enabled. Any other request will turn off the node. 
 * @param response success or not. 
 * 
 * @see Command.srv
 * 
 ***********************************************/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
{
    if (req.command == "start")
    {
        start = true;
        std::cout << "[state_machine] START command received. " << std::endl;
        res.ok = true;
    }
    else 
    {
        if ( start )
        {
            std::cout << "[state_machine] STOP command received, preempting the goal. " << std::endl;
            
            // cancel the running goal
            acglobal->cancelAllGoals( ); 
            res.ok = true;
        }
        else
        {
            std::cout << "[state_machine] state machine already stopped. " << std::endl;
            res.ok = false;
        }
        
        start = false;
    }
    return true;
}

// general function for publishing a info msg to Jupyter
void jupyter_pub( bool working, bool failure, bool goal_cancelled, float x, float y, float th, bool success, float time )
{
    rt2_assignment1::JupyterTargetInfo msg;
    
    msg.working = working;
    msg.failure = failure;
    msg.goal_cancelled = goal_cancelled;
    msg.x = 0;
    msg.y = 0;
    msg.th = 0;
    msg.success = success;
    msg.time = time;
    
    global_jupy->publish( msg );
}

// publish the state of the node (working or not working?)
void jupyter_publish_status( bool status )
{
    jupyter_pub( status, false, false, 0, 0, 0, false, -1 );
}

// mission update (the robot is trying to reach the target)
void jupyter_update_mission( rt2_assignment1::GoToPointGoal& goal, float time )
{
    jupyter_pub( true, false, false, goal.x, goal.y, goal.theta, false, time );
}

// cancellation of the target
void jupyter_target_cancelled( )
{
    jupyter_pub( true, false, true, 0, 0, 0, false, -1 );
}

// failure of the mission (other reason)
void jupyter_target_unreached( )
{
    jupyter_pub( true, true, false, 0, 0, 0, false, -1 );
}

// success of the mission
void jupyter_target_reached( rt2_assignment1::GoToPointGoal& goal, float time )
{
    jupyter_pub( true, false, false, goal.x, goal.y, goal.theta, true, time );
}



/********************************************//**
 *  
 * \brief ROS node main - state_machine
 * 
 ***********************************************/
int main(int argc, char **argv)
{
    
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   
   // provided services
   ros::ServiceServer service = n.advertiseService("/user_interface", user_interface);
   
   // client
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
   // action
   actionlib::SimpleActionClient<rt2_assignment1::GoToPointAction> ac( "go_to_point", true );
   acglobal = &ac; // share the access to the action
   ac.waitForServer( );
    
    // infos for Jupyter interface
    ros::Publisher jupy = n.advertise<rt2_assignment1::JupyterTargetInfo>( "/jupyter_mission_info", 1000 );
    global_jupy = &jupy;
   
   // bounds for the random target
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Position p;
    
    // useful
    rt2_assignment1::GoToPointGoal empty_goal;
    
    // init the state of the GUI
    jupyter_publish_status( false );
   
   while( ros::ok() )
   {
    ros::spinOnce();
    
    // execute this only if the node is enabled
    if (start)
    {
        jupyter_publish_status( true );
        
        // ask for a random target
        client_rp.call(rp);
        
        // prepare the request to the movement action
        rt2_assignment1::GoToPointGoal goal;
        goal.x = rp.response.x;
        goal.y = rp.response.y;
        goal.theta = rp.response.theta;
        std::cout << "NEXT TARGET (" << goal.x << ", " << goal.y << ", " << goal.theta << ")" << std::endl;
        
        // the target is requested to the movemet action: the robot starts to move
        ac.sendGoal( goal );
        
        // waiting for the robot to reach the goal unless any preemption request
        bool success = false;
        float timeFromStart = 0.f;
        std::string state = "ACTIVE";
        while( state != "SUCCEEDED" )
        {
            // wait the minimum time
            (ros::Duration(WAITING_DELAY)).sleep();
            
            // update the status of the node
            ros::spinOnce();
            timeFromStart += WAITING_DELAY;
            state = ac.getState( ).toString( );
            
            // evaluate the new state
            if( state == "PREEMPTED" )
            {
                std::cout << "The goal has been cancelled. " << std::endl;
                jupyter_target_cancelled( );
                break;
            }
            else if( timeFromStart > MAX_WAITING_TIME )
            {
                std::cout << "Unable to reach the final position within the deadline. " << std::endl;
                jupyter_target_unreached( );
                break;
            }
            
            // send the status to Jupyter GUI
            jupyter_update_mission( goal, timeFromStart );
            
        }
        success = ( state == "SUCCEEDED" );
        
        if( success )
        {
            std::cout << "Position reached" << std::endl;
            jupyter_target_reached( goal, timeFromStart );
        }
        else
        {
            std::cout << "Final goal not reached" << std::endl;
        }
    }
    else
    {
     jupyter_publish_status( false );   
    }

    
   }
   return 0;
}
