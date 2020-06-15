#include <ros/ros.h>
#include <servo_server/ServoControl.h>
#include <time.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "servo_scan_client_node");

	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<servo_server::ServoControl>("/servo_control");

	servo_server::ServoControl call;

	// start scanning

	for (int i = 1000; i < 8000; i += 1000)
	{
		call.request.Command  = 112;	// "p" == 112
		call.request.On_Off   = 1;	// 1 == 'on'		
		call.request.Channel  = 1;	// pan servo
		call.request.Position = i;	// [100 , 7900]
		call.request.Speed    = 80;	

		// attempt to call server
		if (client.call(call))
			ROS_INFO("Panning camera");
		
		else
		{
			client.waitForExistence();
			client.call(call);
		}

		sleep(2);		

		for (int j = 2000; j < 6100; j += 100)
		{
			call.request.Command  = 112;	// "p" == 112
			call.request.On_Off   = 1;	// 1 == 'on'
			call.request.Channel  = 2;	// tilt servo
			call.request.Position = j;	// [2000, 6000]
			call.request.Speed    = 50;

			// attempt to call server
			if (client.call(call))
				ROS_INFO("Tilting camera");

			else 
			{
				client.waitForExistence();
				client.call(call);
			}
		}
	}

	// return to center
	call.request.Command  = 112;	
	call.request.On_Off   = 1;			
	call.request.Channel  = 1;	
	call.request.Position = 4000;	
	call.request.Speed    = 80;

	sleep(2);

	call.request.Command  = 112;	
	call.request.On_Off   = 1;			
	call.request.Channel  = 2;	
	call.request.Position = 4000;	
	call.request.Speed    = 80;

	sleep(2);

	return 0;
}

void sleep(int sec)
{
	// converting time into milliseconds
	int milli_seconds = 1000 * sec;
	
	// storing start time
	clock_t start_time = clock();
	
	// looping till 
	while (clock() < start_time + milli_seconds)
		;	
}







