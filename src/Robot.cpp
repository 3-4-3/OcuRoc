// implements a class to handle the 3D robot model

#include <Robot.h>
#include <OgreEntity.h>
#include <tf/transform_datatypes.h>

Robot::Robot(Ogre::SceneManager *mSceneMgr) {
	// set everything up for display
	robot = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	robot->setPosition(Ogre::Vector3(0.0f, 1.0f, 0.0f));
	//robot->attachObject(mSceneMgr->createEntity("Rosie.mesh"));
	/* Good for debugging: add some coordinate systems */
	 robot->attachObject(mSceneMgr->createEntity("CoordSystem"));
				//robot->flipVisibility(); //////////////////// carlos
}

Robot::~Robot() { }

void Robot::updateFrom(tf::TransformListener *tfListener) {
	using namespace Ogre;
	static tf::StampedTransform baseTF;
	static Vector3 translation = Vector3::ZERO;
	static Quaternion orientation = Quaternion::IDENTITY;
	static Quaternion qYn90 = Quaternion(Degree(90), Vector3::UNIT_Y);
	static tfScalar yaw,pitch,roll;
	static Matrix3 mRot;
	
	// get the latest robot position and orientation, transform them to Ogre and update the scene node	
	try {
		tfListener->lookupTransform("camera_left","marker",ros::Time(0), baseTF); //////////////////// carlos
		
		/*translation.x = -baseTF.getOrigin().y();
		translation.y = baseTF.getOrigin().z();
		translation.z = -baseTF.getOrigin().x();
		translation = translation + Vector3(0.0f, 1.0f, 0.0f);
		baseTF.getBasis().getEulerYPR(yaw,pitch,roll);
		mRot.FromEulerAnglesYXZ(Radian(yaw),Radian(0.0f),Radian(0.0f));
		orientation.FromRotationMatrix(mRot);
		orientation = qYn90*orientation;*/
		
		// positioning
		translation.x = baseTF.getOrigin().x();
		translation.y = -baseTF.getOrigin().y();
		translation.z = -baseTF.getOrigin().z();
		// rotation (at least get it into global coords that are fixed on the robot)
		baseTF.getBasis().getEulerYPR(yaw,pitch,roll);
		mRot.FromEulerAnglesXYZ(Radian(roll),Radian(pitch),Radian(yaw));
		orientation.FromRotationMatrix(mRot);
		
		robot->setPosition(translation);
        robot->setOrientation(orientation);
		
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

Ogre::SceneNode* Robot::getSceneNode() {
	return robot;
}
