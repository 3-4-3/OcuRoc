
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <math.h>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"


class FLC 

{
public:
	FLC(void);
	/**< Default constructor. Initialize all members to default values. */
	
	~FLC(void);
	/**< Default destructor. Final cleanup of elements. */

	void moveRobot(double inputVelocityF, double inputVelocityS, double angle);

protected:

	ros::NodeHandle* hRosNode;

	std_msgs::Float32 rotation;        	// Angle that the camera have with respect to the front part of the robot
	std_msgs::Float32 inputVelocityF;   	// Speed in the direction of the camera that the user wants to apply
	std_msgs::Float32 inputVelocityS;   	// Speed in perpendicular direction of the camera that the user wants to apply

	ros::Publisher *fbVel_pub;   		// Forward Speed
  	ros::Publisher *lrVel_pub;   		// Right/left Speed
  	ros::Publisher *rotVel_pub; 		// Rotational speed
  	
  	ros::Publisher *ubot_pub; 		// Rotational speed

};
