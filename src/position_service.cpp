/********************************************//**
 *  
 * \file position_service.cpp
 * <div><b>ROS Node Name</b> 
 *      <ul><li>random_position_server</li></ul></div>
 * \brief Service for Random Pose
 * 
 * \authors Carmine Tommaso Recchiuto
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * The service provided by this service is simple: it can return a (x, y)
 * position randomly generated using bounds specified in the request. <br>
 * The service can generate also an angle around Z axis, already
 * noramlized in <i>[-pi, pi]</i>. 
 * </p>
 * 
 * <b>UML component</b><br>
 * (See -- the overal architecture, for further informations)<br>
 * <img src="#" alt="TODO UML components of position_service.cpp"/><br>
 * 
 * <b>Services:</b> <br>
 * <ul>
 *     <li>
 * 			<i>/position_server</i> : RandomPosition.srv <br>
 * 			Ask for a new random plane position (with bounds) and 
 * 			orientation about Z axis (no bounds, normalized).
 * 		</li>
 * </ul>
 * 
 * <b>TODOs</b><br>
 * \todo UML components of position_service
 * \todo document the functions in the file position_service.cpp
 * 
 ***********************************************/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

#include <cmath>



/********************************************//**
 *  
 * \brief generate a random number between a minimum and a maximum. 
 * 
 * @param M the minimum value, included
 * @param N the maximum value, included
 * 
 * @returns random number in [M, N]
 * 
 ***********************************************/
double randMToN(double M, double N)
{     
	return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
}



/********************************************//**
 *  
 * \brief implementation of service "/position_server"
 * 
 * @param request  the bounds for the planar coordinates
 * @param response a pose {x, y, th_z}
 * 
 * @see RandomPosition.srv
 * 
 ***********************************************/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res)
{
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-M_PI, M_PI);
    
    return true;
}



/********************************************//**
 *  
 * \brief ROS node main - position_service.cpp
 * 
 * A trivial main function: init the node, advertise the service, then 
 * spin. NO OUTPUT on the screen is provided. 
 * 
 ***********************************************/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   
   ros::ServiceServer service = n.advertiseService("/position_server", myrandom);
   
   ros::spin();

   return 0;
}
