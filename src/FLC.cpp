#include "FLC.h"


#define MAX_SPEED 0.1
#define PI 3.1415

FLC::FLC(void)

{
	hRosNode = new ros::NodeHandle();
	
	fbVel_pub = new ros::Publisher(hRosNode->advertise<std_msgs::Float32>("vrep/ubotFBS", 1000));	// Forward Speed
  	lrVel_pub = new ros::Publisher(hRosNode->advertise<std_msgs::Float32>("vrep/ubotLRS", 1000)); 	// Right/left Speed
  	rotVel_pub = new ros::Publisher(hRosNode->advertise<std_msgs::Float32>("vrep/ubotROTS", 1000));	// Rotational speed
	ubot_pub = new ros::Publisher(hRosNode->advertise<geometry_msgs::Twist>("cmd_vel", 1000));
}



FLC::~FLC(void)
{



}


void FLC::moveRobot(double inputVelocityF, double inputVelocityS, double angle) {

	#if false
	
	
    std_msgs::Float32 fbSpeed;
    std_msgs::Float32 lrSpeed;
    std_msgs::Float32 rotSpeed;

	geometry_msgs::Twist twist;
    geometry_msgs::Vector3 lin, ang;
    
    lin.x = MAX_SPEED*(inputVelocityF*cos(angle) + inputVelocityS*sin(angle));
    lin.y = MAX_SPEED*(inputVelocityF*sin(angle) + inputVelocityS*cos(angle));
    
    while(angle > PI) angle -= 2*PI;
    while(angle < -PI) angle += 2*PI;
    
    ang.x = 0;
    ang.y = 0;
    ang.z = 0;
    if(inputVelocityF != 0)
    ang.z = 0.3*angle;
    
    twist.linear = lin;
    twist.angular = ang;
    /**
     * Operations to send the output by taking the input into account
     * 
     */
    // ROT SPEED
    //rotSpeed.data = inputRotation.data;
    // FB SPEED
    //fbSpeed.data = inputVelocityF*cos(angle) + inputVelocityS*sin(angle);
    // LR SPEED
    //lrSpeed.data = inputVelocityF*sin(angle) + inputVelocityS*cos(angle);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //fbVel_pub->publish(fbSpeed);
    //lrVel_pub->publish(lrSpeed);
    //rotVel_pub->publish(rotSpeed);
    ubot_pub->publish(twist);
	
	#else
	
	
	std_msgs::Float32 fbSpeed;
    std_msgs::Float32 lrSpeed;
    std_msgs::Float32 rotSpeed;

	geometry_msgs::Twist twist;
    geometry_msgs::Vector3 lin, ang;
    

    
    while(angle > PI) angle -= 2*PI;
    while(angle < -PI) angle += 2*PI;
    
    
    double L = 0.12;
    double d = 0.3;
    
    
    
    
    double v_l = ((inputVelocityF-inputVelocityS*d/(2*L))*cos(angle) - (inputVelocityS + inputVelocityF*d/(2*L))*sin(angle))*MAX_SPEED;
	double v_r = ((inputVelocityF+inputVelocityS*d/(2*L))*cos(angle) - (inputVelocityS - inputVelocityF*d/(2*L))*sin(angle))*MAX_SPEED;

	double highDesTrackSpeed = std::max(abs(v_l), abs(v_r));
    if(highDesTrackSpeed > MAX_SPEED) {
        v_l = v_l * MAX_SPEED / highDesTrackSpeed;
        v_r = v_r * MAX_SPEED / highDesTrackSpeed;
	}
	
	lin.x = (v_r + v_l)/2.0;
	lin.y = 0.0;
	ang.z = (v_r - v_l)/d;
    
    twist.linear = lin;
    twist.angular = ang;
    /**
     * Operations to send the output by taking the input into account
     * 
     */
    // ROT SPEED
    //rotSpeed.data = inputRotation.data;
    // FB SPEED
    //fbSpeed.data = inputVelocityF*cos(angle) + inputVelocityS*sin(angle);
    // LR SPEED
    //lrSpeed.data = inputVelocityF*sin(angle) + inputVelocityS*cos(angle);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //fbVel_pub->publish(fbSpeed);
    //lrVel_pub->publish(lrSpeed);
    //rotVel_pub->publish(rotSpeed);
    ubot_pub->publish(twist);
	
	
	
	#endif

}

