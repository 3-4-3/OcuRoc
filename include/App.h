#include <stdio.h>      				/* printf */
#include <time.h>       				/* time_t, struct tm, difftime, time, mktime */
#include <tf/transform_listener.h>		// Transforms

#include <OgreCamera.h>					// Ogre
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include "OgreManualObject.h"

#include <cmath>


# define NUMBER_CP     	6 				/* Number of check points */  
# define NUMBER_LAPS   	3 				/* Number of laps */  

// CIRCUIT
# define CIR_WIDE		0.1
# define CIR_HEIGHT		0.075

// GOAL
# define NUMBER_SQ_W	8
# define NUMBER_SQ_H	4
# define HEIGHT_GOAL	0.3


class App {
	protected:
	
	Ogre::SceneManager* mSceneMgr;
	Ogre::SceneNode *objective;
	Ogre::SceneNode *race_cir_i, *race_cir_e, *goalp, *goal;
	
	// Needed for counting the time
	time_t timer; 			// Time of the start of the app from the goal 
	time_t timer_end; 		// Moment the robot perform the amount of laps required
	
	// State of the app
	bool begin;
	bool end;
	bool circ; // If the race circuit was already created
	
	// Point in the race that the robot is found
	int laps;
	int checkPoint;

	// Prev position of the robot (RIGHT NOW UNUSED)
	double x_robot_prev,  z_robot_prev;
	
	// Points of the race circuit
	double circ_int_x[NUMBER_CP], circ_int_z[NUMBER_CP],	// Interior (clockwise dir)
			circ_ext_x[NUMBER_CP], circ_ext_z[NUMBER_CP];	// Exterior (clockwise dir)
	
	
	// Helper functions
	bool intersect(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3);
	bool isCloseFX(double x0, double y0, double x1, double y1);
	bool isClose(double x0, double y0, double x1, double y1, double r);
	


	public:
	
	App (Ogre::SceneManager* mSceneMgr, Ogre::SceneNode *objective);
	
	// Function that is called when the race is started
	void start(tf::TransformListener *tfListener);
	// Called in every cycle of the app
	void step(tf::TransformListener *tfListener);
		
	// Getters
	double getLaps()		{ return laps; }
	double getCP() 		{ return checkPoint; }
	int getTime()	{ 
		if(!begin) return 0;
		if(end) return difftime(timer_end,timer);
		time_t now;
		time(&now);
		return difftime(now,timer); }
};
