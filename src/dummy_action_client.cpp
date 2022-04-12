/********************************************//**
 *  
 * \file dummy_action_client.cpp
 * <div><b>ROS Node Name</b> 
 *      <ul><li>DummyActionClient</li></ul></div>
 * \brief test of the functionalities of \ref go_to_point.py movement action.
 * 
 * \authors Francesco Ganci (S4143910)
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * This simple node aims at testing how the movement action implemented in 
 * the node go_to_point.py works. In particular, cancellation is tested 
 * out. <br><br>
 * The code inside this test is noteworthy, because the action client is
 * implemented as a class which collects also the other service needed. In
 * other words, the class provides a helpful <i>abstraction</i>. 
 * </p>
 * 
 * <b>Clients:</b> <br>
 * <ul>
 *     <li>
 * 			<i>/position_server</i> : RandomPosition.srv <br>
 * 			See the node \ref position_service.cpp
 * 		</li>
 * </ul>
 * 
 * <b>Using actions</b> <br>
 * <ul>
 *     <li>
 * 			<i>go_to_point</i> : GoToPoint.action <br>
 *			See \ref go_to_point.py
 * 		</li>
 * </ul>
 * 
 * <b>TODOs</b><br>
 * \todo TRANSLATE THE LOGS HERE.
 * \todo write the documentation of the code within this node. 
 * 
 ***********************************************/

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "rt2_assignment1/GoToPointAction.h"
#include "rt2_assignment1/RandomPosition.h"




class DummyActionClient
{
public:
	
	// creazione del client e istanziazione dei servizi per ottenere la posizione random
	DummyActionClient( ros::NodeHandle* nh = nullptr ) :
		ac( "go_to_point", true )
	{
		// richiesta dell'action client
		std::cout << "richiesta action client 'go_to_point' ..." << std::endl;
		ac.waitForServer();
		std::cout << "action client 'go_to_point' -> OK" << std::endl;
		
		// richiesta del server per le random positions
		if( nh != nullptr )
		{
			std::cout << "richiesta service 'position_server' ..." << std::endl;
			randomPos = nh->serviceClient<rt2_assignment1::RandomPosition>("/position_server");
			std::cout << "service 'position_server' -> OK" << std::endl;
		}
		else
		{
			std::cout << "richiesta service 'position_server' ..." << std::endl;
			randomPos = innerNodeHandle.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
			std::cout << "service 'position_server' -> OK" << std::endl;
		}
	}
	
	// invio del goal
	void sendRandomGoal( )
	{
		rt2_assignment1::GoToPointGoal goal = generateRandomGoal();
		
		// invia all'ActionServer il goal
		ac.sendGoal( goal );
	}
	
	// attesa
	void waitFor( float duration )
	{
		(ros::Duration( duration )).sleep();
	}
	
	// cancella la richiesta
	void cancelRequest( )
	{
		ac.cancelAllGoals();
	}
	
private:
	// actlion client da cancellare 'go_to_point'
	actionlib::SimpleActionClient<rt2_assignment1::GoToPointAction> ac;
	
	// servizio 'position_server'
	ros::ServiceClient randomPos;
	
	// noe handle
	ros::NodeHandle innerNodeHandle;
	
	// richiedi un goal al service
	rt2_assignment1::GoToPointGoal generateRandomGoal( )
	{
		// richiesta della posizione
		rt2_assignment1::RandomPosition rp;
		rp.request.x_max = 5.0;
		rp.request.x_min = -5.0;
		rp.request.y_max = 5.0;
		rp.request.y_min = -5.0;
		randomPos.call(rp);
		
		// goal
		rt2_assignment1::GoToPointGoal goal;
		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		
   		return goal;
	}
};


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "DummyActionClient" );
	ros::NodeHandle nh;
	
	//DummyActionClient dac( &nh );
	DummyActionClient dac;
	
	std::cout << "invio del goal ..." << std::endl;
	dac.sendRandomGoal( );
	
	std::cout << "attesa di 2s ..." << std::endl;
	dac.waitFor( 2 );
	
	std::cout << "cancellazione della richiesta ..." << std::endl;
	dac.cancelRequest( );
	
	std::cout << "fatto!" << std::endl;
	return 0;
}
