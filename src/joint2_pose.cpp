//Headers: give access to ros libraries and msgs types
#include <ros/ros.h>
#include <std_msgs/UInt16.h>

int main(int argc, char** argv)
{
	// --- ROS Setup --- //
	
	ros::init(argc,argv,"joint2_pose");									// Initializes backend of ROS
	
	ros::NodeHandle node_handle;										// Gives a variable to interact with backend of ROS
	
	ros::Publisher publisher = node_handle.advertise<std_msgs::UInt16>("joint2_angle_topic",1);		// Publisher to interact with a topic using a data type, name of a topic, and queue size
	
	// --- Loop --- //
	ros::Rate rate(3); //Hz										// 3 times per second
	int count = 1;
	
	while(ros::ok())											// Should the application still be running
	{
		
		std_msgs::UInt16 angle;									// constructing an angle in which to rotate to
		if(count % 2 == 0) { 		// even counter
			angle.data = 30;
			count = 1;
		}
		else{ 				// odd counter
			angle.data = 0;
			count = 2;
		}

		publisher.publish(angle);									// Publish the angle to the topic from above
		
		ros::spinOnce();										// Give the opportunity for backend to take care of any events that have/need to happen
		
		rate.sleep();											// Sleep for a dynamic time to achive refesh rate specified
	}
	
	return 0;
}
