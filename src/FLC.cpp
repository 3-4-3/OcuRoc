#include "FLC.h"



FLC::FLC(void)

{
	hRosNode = new ros::NodeHandle();
	
	fbVel_pub = new ros::Publisher(hRosNode->advertise<std_msgs::Float32>("vrep/ubotFBS", 1000));	// Forward Speed
  	lrVel_pub = new ros::Publisher(hRosNode->advertise<std_msgs::Float32>("vrep/ubotLRS", 1000)); 	// Right/left Speed
  	rotVel_pub = new ros::Publisher(hRosNode->advertise<std_msgs::Float32>("vrep/ubotROTS", 1000));	// Rotational speed

}



FLC::~FLC(void)
{


}


void FLC::moveRobot(double inputVelocityF, double inputVelocityS, double angle) {

    std_msgs::Float32 fbSpeed;
    std_msgs::Float32 lrSpeed;
    std_msgs::Float32 rotSpeed;

    
    /**
     * Operations to send the output by taking the input into account
     * 
     */
    // ROT SPEED
    //rotSpeed.data = inputRotation.data;
    // FB SPEED
    fbSpeed.data = inputVelocityF*cos(angle) + inputVelocityS*sin(angle);
    // LR SPEED
    lrSpeed.data = inputVelocityF*sin(angle) + inputVelocityS*cos(angle);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    fbVel_pub->publish(fbSpeed);
    lrVel_pub->publish(lrSpeed);
    //rotVel_pub->publish(rotSpeed);



}

