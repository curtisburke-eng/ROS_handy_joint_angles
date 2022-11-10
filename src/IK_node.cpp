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
#include <math.h>                                                       // cmath: cos, sin, atan2, pow, etc.

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

bool computeIK(float x, float y, float phi_deg) {
/* 
*   computeIK completes the math to translate  and x & y coord. to joint angles
*       of a 3DOF planar arm. 
*       INPUTS: x coord. y coord. and desired angle of link 3 (in deg)
*       OUTPUTS: boolean describing if the target is reachable given the inputs
*/

    // Convert to radians
    float phi = (phi_deg * M_PI)/180 ;                                   // radians

    /* 
    Equations for Inverse kinematics (3DOF Planar)
        x = L_1*cos(theta_1) + L_2*cos(theta_1 + theta_2) + L_3*cos(theta_1 + theta_2 + theta_3)
        y = L_1*sin(theta_1) + L_2*sin(theta_1 + theta_2) + L_3*sin(theta_1 + theta_2 + theta_3)
        phi = theta_1 + theta_2 + theta_3
    */

    // Rearrange & subsitute
    float x_2dof = x - L_3*cos(phi);
    float y_2dof = y - L_3*sin(phi);
    
    // Compute the sum of squres (sos) for the 2_dof section of the eqns
    float sos = pow(x_2dof,2) + pow(y_2dof,2);

    // Expand trig functions & combine like terms
    // sos = 2*L_1*L_2*cos(theta_2) + L_1^2 + L_2^2
    // Rearange (Solving for cos(theta_2))
    float c2 = (sos - pow(L_1,2) - pow(L_2,2))/(2*L_1*L_2);

    // Using identity: sin^2(theta) + cos^2(theta) = 1
    // Solve for sin
    float s2 = sqrt(1-pow(c2,2));                                       // elbow down

    // Using atan2 with sin and cos
    // Solve for theta_2
    float theta_2 = atan2(s2, c2);

    // ---- TODO: COMMENT ON HOW THIS HAPPEND ----
    
    // Solve for sin(theta_1)
    float s1 = ((L_1 + L_2*c2)*y_2dof - L_2*s2*x_2dof)/sos;

    // Solve for cos(theta_1)
    float c1 = ((L_1 + L_2*c2)*x_2dof + L_2*s2*y_2dof)/sos;

    // Using atan2 with sin and cos
    // Solve for theta_1
    float theta_1 = atan2(s1,c1);

    // Rearrage 3rd kinematic eqn
    // Solve for theta_3
    float theta_3 = phi - theta_1 - theta_2;

    // Check that soln is real
    if(!isnormal(theta_1)) { return false; }
    if(!isnormal(theta_2)) { return false; }
    if(!isnormal(theta_3)) { return false; }

    // Convert radians to Degrees & insert into array
    joint_angles[0] = static_cast<int>(theta_1*180/M_PI);
    joint_angles[1] = static_cast<int>(theta_2*180/M_PI);
    joint_angles[2] = static_cast<int>(theta_3*180/M_PI);

    // check that soln is w/in the workspace
    // -90 <= theta_1 <=90
    // -90 <= theta_2 <=90
    // -90 <= theta_3 <=90
    for(int i=0; i<3; i++) {
        if(joint_angles[i] < -90 || joint_angles[i] > 90) { return false; }
    }

    // If solution is found (and not caught by valid checks)
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
