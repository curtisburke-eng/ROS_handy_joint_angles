/*
*   IK_node.cpp
*   Author: Curtis Burke
*   Project: Life-sized Mr. Handy Robot
*   Description: 
*       This ROS node functions as the main computation node for Inverse Kinematics (IK) for a 
*       3DOF robot arm used as an appendage of the Life Sized Mr. Handy Robot 
*       It attempts to compute, return, and publish the joint angles needed to reach a specified
*       x & y coordinate. It also attempts to maintain a verticle Link #3 (ie phi == 90); however,
*       the orientation will be changed to achieve the target (x,y) point. 
*
*/


// Include Libraries & Headers
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <math.h>                                                       // cos, sin, etc

// ------------------------------------------------------------------------
// Setup Global Variables
// ------------------------------------------------------------------------

// Joint Angle Output Array
// [theta_1, theta_2, theta_3, theta_E]
int joint_angles[4];
int *ja_ptr = joint_angles;

// Link Lengths
float L_1 = 9;
float L_2 = 9;
float L_3 = 4;
float L_E = 2;

// ------------------------------------------------------------------------
// Declare & Define Supporting Fuctions
// ------------------------------------------------------------------------

bool computeIK(int px, int py, int phi_deg) {
/* 
*   computeIK completes the math to translate  and x & y coord. to joint angles
*       of a 3DOF planar arm. 
*       INPUTS: x coord. y coord. and desired angle of link 3 (in deg)
*       OUTPUTS: boolean describing if the target is reachable given the inputs
*/

    // Convert to radians
    float phi = (phi_deg * pi)/180 ; // radians

    // Equations for Inverse kinematics
    wx = x - L_3*cos(phi);
    wy = y - L_3*sin(phi);

    // TODO: EDIT BELOW (convert from python to C++)
    /*
    delta = wx**2 + wy**2
    c2 = ( delta -a1**2 -a2**2)/(2*a1*a2)
    s2 = sqrt(1-c2**2)  # elbow down
    theta_2 = arctan2(s2, c2)

    s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
    c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
    theta_1 = arctan2(s1,c1)
    theta_3 = phi-theta_1-theta_2

    print('theta_1: ', rad2deg(theta_1))
    print('theta_2: ', rad2deg(theta_2))
    print('theta_3: ', rad2deg(theta_3))
    */

    // Check that soln is real

    // check that soln is w/in the workspace
    // -90 <= theta_1 <=90
    // -90 <= theta_2 <=90
    // -90 <= theta_3 <=90
   
    // If solution is found
    return true;
    // Else: If no solution is found 
    // return false;

}

// ------------------------------------------------------------------------
// Declare & Define Main Function
// ------------------------------------------------------------------------
int main(int argc, char** argv)
{
	// ----- ROS SETUP -----
	ros::init(argc,argv,"IK_node");								        // Initializes backend of ROS
	ros::NodeHandle node_handle;										// Gives a variable to interact with backend of ROS

    ros::Rate rate(2); //Hz										        // 2 times per second

    // TODO: Publish an array not a single value
	ros::Publisher publisher = node_handle.advertise<std_msgs::UInt16>("joint_angles_array_topic",1);		// Publisher to interact with a topic using a data type, name of a topic, and queue size
	
    // ----- PROGRAM SETUP -----

    // TODO: subscribe to topic from an input node with coord.
    // Input coordinate of end effector 
    float x = 15;
    float y = 0;

    // Previous coordinate 
    float x_old = x;
    float y_old = y;

    // Home Location (x,y coord. for the resting point of the EE)
    float home_x = 10;
    float home_y = -10;

    // Input Orientation of End Effector
    float phi_deg = 90;                                                     // Desired orientation of EE (when compared to the x0 axis)
    float phi_step = 5;                                                     // Amount of degrees to change in the event the target is unreachable at desired orientation
    
    // Solution Tracking
    bool reachable = 0;                                                     // Track if a target can be reached with current arm and given orientation

	// ----- MAIN LOOP -----
	while(ros::ok())											            // Loop while the application should still be running (via ROS)
	{
        // TODO: Subscribe to topic and grab input

        // Check if input has changed (New input != current input)
        if(x != x_old || y != y_old){ // a new input has arrived
            // Reset solution tracker
            reachable = 0;
            // Reset the desired EE orientation
            phi_deg = 90;
        }

        // Loop through orientations of the EE to find solution closest to phi == 90deg
        while(!reachable){ // while solution is not found

            // Compute the Inverse Kinematics & Determine if Solution is valid
            reachable = computeIK(x, y, phi_deg);

            // Update orientation of EE for next attempt
            if(!reachable) {
                if(x < home_x) {
                    phi_deg = phi_deg + phi_step
                }
                else { // x > home_x
                    phi_deg = phi_deg - phi_step
                }
            }

            
            //if reachable: Break out of loop

            /*
            TODO: should it be a certain number of attempts (instead of while(!reachable) do a while(count <= threshold))
                threshold could be based on the number of attepts possible with the phi_step while still keeping the arm in 
                elbow down orientation (180 <= phi >= 0) starting at phi == 90 with step of 1 there could be 90 attempts before
                arm went negative.
            */
        }

        // Update old target values to give room for new ones
        x_old = x;
        y_old = y;
    
        if(reachable) {
            // Send out computed values 
            publisher.publish(joint_angles);								// Publish the angle to the topic from above
        }		
		
		ros::spinOnce();										            // Give the opportunity for backend to take care of any events that have/need to happen
		
		rate.sleep();											            // Sleep for a dynamic time to achive refesh rate specified
	}
	
	return 0;
}