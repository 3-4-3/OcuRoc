#ifndef _ROBOT_H_
#define _ROBOT_H_


#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <tf/transform_listener.h>

/**< \brief Represents a Robot.
 * This class handles the robot avatar.
 */
class Robot {
public:
	/**  Yaw difference between Yaw in t and Yaw in t-1 (t=now) */
	///double yaw_difference;

	Robot(Ogre::SceneManager*, Ogre::SceneNode*);
	/**< Set up the avatar and prepare the scene.*/
	~Robot();
	/**< Default destructor.*/
	
	virtual void updateFrom(tf::TransformListener*);
	/**< Update the Robot position and orientation from the given TransformListener (ROS).*/
	virtual Ogre::SceneNode* getSceneNode();
	/**< Returns the scene node of the avatar.*/
protected:
	Ogre::SceneNode *robot; /**< The scene node of the avatar.*/
};
 #endif
