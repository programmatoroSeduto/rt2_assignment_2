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
 * The node allows also cancel the action: see the description of the 
 * service /user_interface. <br><br>
 * In general the node, when enabled, works in this way. First of all, if
 * a new target is obtained from the service /user_interface; 
 * then, the node sends the request to the movement action. 
 * After sent the request, the node executes a check upon the status
 * of the movement action every <i>at least</i> 0.1 seconds (see 
 * \ref WAITING_DELAY) for maximum 30 seconds (see \ref MAX_WAITING_TIME).
 * If the action requires too much time, the request is cancelled. <br><br>
 * Note that the bounds of the target are provided once in the main function:
 * x in [-5.0, 5.0], y in [-5.0, 5.0]. <br><br>
 * 
 * Communication with Jupyter is implemented through a dedicated topic 
 * of type JupyterTargetInfo. 
 * </p>
 * 
 * <b>Publishers:</b> <br>
 * <ul>
 *     <li>
 *          <i>/jupyter_mission_info</i> : JupyterTargetInfo.msg <br>
 *          Updates about the running mission for Jupyter. 
 *      </li>
 * </ul>
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
 * \todo the command should be case-insensitive
 * \todo review the output to the screen
 * 
 ***********************************************/

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "rt2_assignment_2/Command.h"
#include "rt2_assignment_2/Position.h"
#include "rt2_assignment_2/RandomPosition.h"
#include "rt2_assignment_2/GoToPointAction.h"
#include "rt2_assignment_2/JupyterTargetInfo.h"

#include <string>



/// The minimom delay between one cycle and the next one
#define WAITING_DELAY 0.1



/// The maximum waiting time before the forced break of the cycle
#define MAX_WAITING_TIME 30



/// when true, the node is enabled. 
bool start = false;



/// Shared pointer to the movement action handler
actionlib::SimpleActionClient<rt2_assignment_2::GoToPointAction>* acglobal;


/// shared pointer to the info topic towards Jupyter
ros::Publisher* global_jupy;



/********************************************//**
 *  
 * @brief implementation of service <b>/user_interface</b>
 * 
 * @param req The command to execute. If the command is the string "start"
 *          then the node is enabled. Any other request will turn off the node. 
 * @param res success or not. 
 * 
 * @see Command.srv
 * 
 ***********************************************/
bool user_interface(rt2_assignment_2::Command::Request &req, rt2_assignment_2::Command::Response &res)
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



/********************************************//**
 *  
 * @brief general function for publishing a info msg to Jupyter
 * 
 * This function allows to make the update message and publish it to 
 * Jupyter quickly, just passing the fields of the message as parameters. 
 * 
 * @param working if the random behaviour is active or not
 * @param failure if the mission failed or not
 * @param goal_cancelled if the goan has been cancelled or not
 * @param x the 'x' target component
 * @param y the 'y' target component
 * @param th the head angle in the target
 * @param success if the mission succeeded or not
 * @param time how much time the mission did last until now
 * 
 * @note don't use this function directly, because here there's no
 * 		way to ensure the coherence of the fields each other. Use the
 * 		wrappers. 
 * 
 ***********************************************/
void jupyter_pub( bool working, bool failure, bool goal_cancelled, float x, float y, float th, bool success, float time )
{
    rt2_assignment_2::JupyterTargetInfo msg;
    
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



/********************************************//**
 *  
 * @brief publish the state of the node (working or not working?)
 * 
 * Function called when the random behaviour is turned on/off. 
 * 
 * @param status the new status of the node (workin or not?)
 * 
 ***********************************************/
void jupyter_publish_status( bool status )
{
    jupyter_pub( status, false, false, 0, 0, 0, false, -1 );
}



/********************************************//**
 *  
 * @brief mission update (the robot is trying to reach the target)
 * 
 * while the robot is moving towards the goal nder the random behaviour,
 * the node regularly sends a notification to the GUI to make it aware 
 * about how the mission is going on. 
 * 
 * @param goal the objective
 * @param time how much time elapsed from the beginning of the mission
 * 
 ***********************************************/
void jupyter_update_mission( rt2_assignment_2::GoToPointGoal& goal, float time )
{
    jupyter_pub( true, false, false, goal.x, goal.y, goal.theta, false, time );
}



// 
/********************************************//**
 *  
 * @brief cancellation of the target
 * 
 * called when the target is cancelled through an external request. 
 * 
 ***********************************************/
void jupyter_target_cancelled( )
{
    jupyter_pub( true, false, true, 0, 0, 0, false, -1 );
}



/********************************************//**
 *  
 * @brief failure of the mission (other reason)
 * 
 * the objective can't be reached due to some other reason.
 * Typically, the node <i>state_machine</i> interrupts the mission
 * when the maximum time expired. 
 * 
 ***********************************************/
void jupyter_target_unreached( )
{
    jupyter_pub( true, true, false, 0, 0, 0, false, -1 );
}



/********************************************//**
 *  
 * @brief success of the mission
 * 
 * A message is sent to Jupyter when the mission ends
 * with success, containing the goal and the time elapsed.
 * 
 * @param goal the objective
 * @param time how much time elapsed from the beginning of the mission
 * 
 ***********************************************/
void jupyter_target_reached( rt2_assignment_2::GoToPointGoal& goal, float time )
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
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment_2::RandomPosition>("/position_server");
   
   // action
   actionlib::SimpleActionClient<rt2_assignment_2::GoToPointAction> ac( "go_to_point", true );
   acglobal = &ac; // share the access to the action
   ac.waitForServer( );
    
    // infos for Jupyter interface
    ros::Publisher jupy = n.advertise<rt2_assignment_2::JupyterTargetInfo>( "/jupyter_mission_info", 1000 );
    global_jupy = &jupy;
   
   // bounds for the random target
   rt2_assignment_2::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment_2::Position p;
    
    // useful
    rt2_assignment_2::GoToPointGoal empty_goal;
    
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
        rt2_assignment_2::GoToPointGoal goal;
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
