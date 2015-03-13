#include "App.h"



App::App (Ogre::SceneManager* mSceneMgr, Ogre::SceneNode *objective) { 			// Constructor

	this->mSceneMgr = mSceneMgr;
	this->objective = objective;
	
	this->objective->setVisible(false);
	circ  = false;

}


void App::start(tf::TransformListener *tfListener) {		// Put parameters to 0
	
	// Initialice app parameters
	begin 		= false;
	end 		= false;
	laps		= 0;
	checkPoint	= 0;
	time(&timer);
	
	// Shows the objective for the robot
	this->objective->setVisible(true);
	
	static tf::StampedTransform baseTF;
	
	// Saves the position of the robot
	try {
		tfListener->lookupTransform("map","you_bot",ros::Time(0), baseTF);

		x_robot_prev = baseTF.getOrigin().x();
		z_robot_prev = baseTF.getOrigin().z();

	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	
	
	/**
	 * Creates race circuit
	 * 
	 */
	if(!circ) {
	try {
		// Saves the points for the race circuit
		double circ_x[NUMBER_CP], circ_z[NUMBER_CP];
		for(int i = 0; i < NUMBER_CP; i++) {
			
			std::ostringstream num;	
			num <<  i;
			tfListener->lookupTransform("map","cp_"+num.str(),ros::Time(0), baseTF);
			
			circ_x[i] = baseTF.getOrigin().x();
			circ_z[i] = baseTF.getOrigin().z();
		}
		
		// Calculates the interior and exterior points
		for(int i = 0; i < NUMBER_CP; i++) {
			
			// Normal vector P(i-1)-P(i)
			double x_normal_s0, z_normal_s0;
			if(i!=0) {
				x_normal_s0 = -(circ_z[i] - circ_z[i-1]);
				z_normal_s0 = circ_x[i] - circ_x[i-1];
			} else {
				x_normal_s0 = -(circ_z[0] - circ_z[NUMBER_CP-1]);
				z_normal_s0 = circ_x[0] - circ_x[NUMBER_CP-1];
			}
			
			std::cout << " x z i " << circ_x[i] << " " << circ_z[i] << std::endl;
			std::cout << " x z i-1 " << circ_x[i-1] << " " << circ_z[i-1] << std::endl;
			std::cout << " x z nor " << x_normal_s0 << " " << z_normal_s0 << std::endl;
			
			// Normalice
			double abs = sqrt(x_normal_s0*x_normal_s0+z_normal_s0*z_normal_s0);
			x_normal_s0 = x_normal_s0/abs;
			z_normal_s0 = z_normal_s0/abs;
			
			
			
			// Normal vector P(i)-P(i+1)
			double x_normal_s1, z_normal_s1;
			if(i!=NUMBER_CP-1) {
				x_normal_s1 = -(circ_z[i+1] - circ_z[i]);
				z_normal_s1 = circ_x[i+1] - circ_x[i];
			} else {
				x_normal_s1 = -(circ_z[0] - circ_z[NUMBER_CP-1]);
				z_normal_s1 = circ_x[0] - circ_x[NUMBER_CP-1];
			}
			
			// Normalice
			abs = sqrt(x_normal_s1*x_normal_s1+z_normal_s1*z_normal_s1);
			x_normal_s1 = x_normal_s1/abs;
			z_normal_s1 = z_normal_s1/abs;
			
			// Sum of both
			double x_sum, z_sum;
			x_sum = x_normal_s0 + x_normal_s1;
			z_sum = z_normal_s0 + z_normal_s1;
			
			// Normalice
			abs = sqrt(x_sum*x_sum+z_sum*z_sum);
			x_sum = x_sum/abs;
			z_sum = z_sum/abs;
			
			// This will make the road the same width everywhere
			double dot_product = x_sum*x_normal_s0 + z_sum*z_normal_s0;
			x_sum = x_sum/dot_product;
			z_sum = z_sum/dot_product;
			
			// And finally put the width we want
			x_sum = CIR_WIDE*x_sum;
			z_sum = CIR_WIDE*z_sum;
			
			// Saves interior and exterior points
			circ_int_x[i] = circ_x[i] - x_sum;
			circ_int_z[i] = circ_z[i] - z_sum;
			circ_ext_x[i] = circ_x[i] + x_sum;
			circ_ext_z[i] = circ_z[i] + z_sum;
			
			std::cout << " x z final " << circ_ext_x[i] << " " << circ_ext_z[i] << std::endl;
			
		}
		
		// Creates the Race Circuit
		Ogre::ManualObject *mPCRender;
		
		// Interior
		mPCRender= mSceneMgr->createManualObject();
		mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
		
		for(int i = 0; i < NUMBER_CP; i++) {
			mPCRender->position(circ_int_x[i], 0.00f, circ_int_z[i]);
			mPCRender->colour(Ogre::ColourValue(0.2f, 0.2f, 0.2f, 0.3f));
			
			mPCRender->position(circ_int_x[i], CIR_HEIGHT, circ_int_z[i]);
			mPCRender->colour(Ogre::ColourValue(0.2f, 0.2f, 0.2f, 0.3f));
		}
		
		for(int i = 0; i < NUMBER_CP-1; i++) {
			mPCRender->triangle(i*2,i*2+1,i*2+2);
			mPCRender->triangle(i*2+1,i*2,i*2+2);
			
			mPCRender->triangle(i*2+1,i*2+2,i*2+3);
			mPCRender->triangle(i*2+2,i*2+1,i*2+3);
		}
		
		// Last strecht
		mPCRender->triangle((NUMBER_CP-1)*2,(NUMBER_CP-1)*2+1,0);
		mPCRender->triangle((NUMBER_CP-1)*2+1,(NUMBER_CP-1)*2,0);
			
		mPCRender->triangle((NUMBER_CP-1)*2+1,0,1);
		mPCRender->triangle(0,(NUMBER_CP-1)*2+1,1);
		
		mPCRender->end();
		mPCRender->convertToMesh("RaceCI"); 
		
		race_cir_i = mSceneMgr->getRootSceneNode()->createChildSceneNode("RaceCI");
		race_cir_i->attachObject(mSceneMgr->createEntity("RaceCI"));
		
		
		// Exterior
		mPCRender= mSceneMgr->createManualObject();
		mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
		
		for(int i = 0; i < NUMBER_CP; i++) {
			mPCRender->position(circ_ext_x[i], 0.00f, circ_ext_z[i]);
			mPCRender->colour(Ogre::ColourValue(0.2f, 0.2f, 0.2f, 0.3f));
			
			mPCRender->position(circ_ext_x[i], CIR_HEIGHT, circ_ext_z[i]);
			mPCRender->colour(Ogre::ColourValue(0.2f, 0.2f, 0.2f, 0.3f));
		}
		
		for(int i = 0; i < NUMBER_CP-1; i++) {
			mPCRender->triangle(i*2,i*2+1,i*2+2);
			mPCRender->triangle(i*2+1,i*2,i*2+2);
			
			mPCRender->triangle(i*2+1,i*2+2,i*2+3);
			mPCRender->triangle(i*2+2,i*2+1,i*2+3);
		}
		
		// Last strecht
		mPCRender->triangle((NUMBER_CP-1)*2,(NUMBER_CP-1)*2+1,0);
		mPCRender->triangle((NUMBER_CP-1)*2+1,(NUMBER_CP-1)*2,0);
			
		mPCRender->triangle((NUMBER_CP-1)*2+1,0,1);
		mPCRender->triangle(0,(NUMBER_CP-1)*2+1,1);
		
		mPCRender->end();
		mPCRender->convertToMesh("RaceCE"); 
		
		race_cir_e = mSceneMgr->getRootSceneNode()->createChildSceneNode("RaceCE");
		race_cir_e->attachObject(mSceneMgr->createEntity("RaceCE"));
		
		
		/**
		 * 	Create the goal
		 * 
		 */
		 
		// Creates the goal posts
		
		mPCRender= mSceneMgr->createManualObject();
		mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_LINE_LIST);
		mPCRender->position(circ_ext_x[0],0.0f,circ_ext_z[0]);
		mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
		mPCRender->position(circ_ext_x[0],HEIGHT_GOAL,circ_ext_z[0]);
		mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
		mPCRender->position(circ_int_x[0],0.0f,circ_int_z[0]);
		mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
		mPCRender->position(circ_int_x[0],HEIGHT_GOAL,circ_int_z[0]);
		mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));


		mPCRender->index(1);
		mPCRender->index(0);
		mPCRender->index(2);
		mPCRender->index(3);
		mPCRender->end();
		mPCRender->convertToMesh("Goalp");
		
		goalp = mSceneMgr->getRootSceneNode()->createChildSceneNode("Goalp");
		goalp->attachObject(mSceneMgr->createEntity("Goalp"));
		
		// Goal
		mPCRender= mSceneMgr->createManualObject();
		mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
		
		double dif_x_goal, dif_z_goal;
		
		dif_x_goal = -(circ_ext_x[0] - circ_int_x[0]);
		dif_z_goal = -(circ_ext_z[0] - circ_int_z[0]);
		
		// Creates the goal with the specified number of columns and rows
		for(int j = 0; j < NUMBER_SQ_H; j++) {
			
			// The goal will contain squares, so the same lenght is required for both sides
			double square_lenght = sqrt(dif_x_goal*dif_x_goal+dif_z_goal*dif_z_goal)/NUMBER_SQ_W;
			
		for(int i = 0; i < NUMBER_SQ_W; i++) {
			
			// BLACK (in first row)
			mPCRender->position(circ_ext_x[0]+dif_x_goal*i/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*(-j-1), circ_ext_z[0]+dif_z_goal*i/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));	// Every row will change colours so they will alternate
			else mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));			//
			mPCRender->position(circ_ext_x[0]+dif_x_goal*i/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*-j, circ_ext_z[0]+dif_z_goal*i/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
			else mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
			
			mPCRender->position(circ_ext_x[0]+dif_x_goal*(0.5+i)/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*(-j-1), circ_ext_z[0]+dif_z_goal*(0.5+i)/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
			else mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
			mPCRender->position(circ_ext_x[0]+dif_x_goal*(0.5+i)/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*-j, circ_ext_z[0]+dif_z_goal*(0.5+i)/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
			else mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
			
			// WHITE (in first row)
			mPCRender->position(circ_ext_x[0]+dif_x_goal*(0.5+i)/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*(-j-1), circ_ext_z[0]+dif_z_goal*(0.5+i)/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
			else mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
			mPCRender->position(circ_ext_x[0]+dif_x_goal*(0.5+i)/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*-j, circ_ext_z[0]+dif_z_goal*(0.5+i)/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
			else mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
			
			
			mPCRender->position(circ_ext_x[0]+dif_x_goal*(1.0+i)/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*(-j-1), circ_ext_z[0]+dif_z_goal*(1.0+i)/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
			else mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
			mPCRender->position(circ_ext_x[0]+dif_x_goal*(1.0+i)/NUMBER_SQ_W, HEIGHT_GOAL-square_lenght*-j, circ_ext_z[0]+dif_z_goal*(1.0+i)/NUMBER_SQ_W);
			if(j%2 == 0) mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
			else mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
			
			
		}
		}
			
		// Creates the triangles from the indexes
		for(int i = 0; i < NUMBER_SQ_H*NUMBER_SQ_W*4; i=i+2) {
			mPCRender->triangle(i*2,i*2+1,i*2+2);
			mPCRender->triangle(i*2+1,i*2,i*2+2);
			
			mPCRender->triangle(i*2+1,i*2+2,i*2+3);
			mPCRender->triangle(i*2+2,i*2+1,i*2+3);
		}
		
		mPCRender->end();
		mPCRender->convertToMesh("Goal"); 
		
		goal= mSceneMgr->getRootSceneNode()->createChildSceneNode("Goal");
		goal->attachObject(mSceneMgr->createEntity("Goal"));
		
		circ = true;
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	}
}

void App::step(tf::TransformListener *tfListener) { 		// Check if robot went to next check point
						// Number of laps
	
	using namespace Ogre;
	static tf::StampedTransform baseTF;

	try {
		tfListener->lookupTransform("map","you_bot",ros::Time(0), baseTF);

		double x_robot = baseTF.getOrigin().x();
		double z_robot = baseTF.getOrigin().z();

		if(!begin) {

			tfListener->lookupTransform("map","cp_0",ros::Time(0), baseTF);

			double x_cp0 = baseTF.getOrigin().x();
			double z_cp0 = baseTF.getOrigin().z();
			
			objective->setPosition(Ogre::Vector3(x_cp0, 0.0f, z_cp0));
			
			if(isCloseFX(x_cp0, z_cp0, x_robot, z_robot)) {
				time(&timer);
				begin = true;
				checkPoint++;
			}

		
			
		} else {

			std::ostringstream num;	
			num <<  (checkPoint);

			tfListener->lookupTransform("map","cp_"+num.str(),ros::Time(0), baseTF);

			double x_cp0 = baseTF.getOrigin().x();
			double z_cp0 = baseTF.getOrigin().z();
			
			objective->setPosition(Ogre::Vector3(x_cp0, 0.0f, z_cp0));

			if(isCloseFX(x_cp0, z_cp0, x_robot, z_robot)) { // Next checkpoint
				
				if(checkPoint == 0) {	// Next lap
					laps++;
					if(laps >= NUMBER_LAPS) { // END of the race
						end = true;
						time(&timer_end);
						this->objective->setVisible(false);
					}
				}

				checkPoint++;
				if(checkPoint >= NUMBER_CP) {
					checkPoint = 0;
				}
			}
			
		}

		x_robot_prev = x_robot;
		z_robot_prev = z_robot;

		

	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
}


/** 
 * P0 Left point/marker of the check point segment
 * P1 Left point/marker of the check point segment
 * P3 Previous robot position
 * P4 Real robot position
 *
 */
bool App::intersect(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {

	// Normal vector to segment P0-P1
	double x_normal_s0, y_normal_s0;
	x_normal_s0 = -(y1 - y0);
	y_normal_s0 = x1 - x0;

	// Normal vector to segment P2-P3
	double x_normal_s1, y_normal_s1;
	x_normal_s1 = -(y3 - y2);
	y_normal_s1 = x3 - x2;

	// Vector from the start of the segment P0-P1 to P2
	double x_s0_p0, y_s0_p0;
	x_s0_p0 = x2 - x0;
	y_s0_p0 = y2 - y0;

	// Vector from the start of the segment P0-P1 to P3
	double x_s0_p1, y_s0_p1;
	x_s0_p1 = x3 - x0;
	y_s0_p1 = y3 - y0;

	// Vector from the start of the segment P2-P3 to P0
	double x_s1_p0, y_s1_p0;
	x_s1_p0 = x0 - x2;
	y_s1_p0 = y0 - y2;

	// Vector from the start of the segment P2-P3 to P1
	double x_s1_p1, y_s1_p1;
	x_s1_p1 = x1 - x2;
	y_s1_p1 = y1 - y2;

	std::cout << " x_s0_p0 " << x_s0_p0 << " x_normal_s0 " << x_normal_s0 << " y_s0_p0 " << y_s0_p0 << " y_normal_s0 " << y_normal_s0 << std::endl;
	std::cout << " P1 " << ((x_s0_p0 * x_normal_s0 + y_s0_p0 * y_normal_s0) < 0) << " P2 " << 
						((x_s0_p1 * x_normal_s0 + y_s0_p1 * y_normal_s0) > 0) << " P3 " <<	
						(((x_s1_p0 * x_normal_s1 + y_s1_p0 * y_normal_s1) * 
						(x_s1_p1 * x_normal_s1 + y_s1_p1 * y_normal_s1)) < 0) << std::endl;
	if ( (x_s0_p0 * x_normal_s0 + y_s0_p0 * y_normal_s0) < 0 &&	// Point P2 before segment P0-P1
		(x_s0_p1 * x_normal_s0 + y_s0_p1 * y_normal_s0) > 0 &&	// Point P3 after segment P0-P1
		((x_s1_p0 * x_normal_s1 + y_s1_p0 * y_normal_s1) * 
		(x_s1_p1 * x_normal_s1 + y_s1_p1 * y_normal_s1)) < 0	// Points P0-P1 in diferent sides of line formed by P2-P3
		) {
		return true;
	} else {
		return false;
	}

}

bool App::isCloseFX(double x0, double y0, double x1, double y1) {

		return isClose( x0,  y0,  x1,  y1, 0.35f);

}

/** 
 * P0 Position of the check point
 * P1 Robot position
 * r radious
 */
bool App::isClose(double x0, double y0, double x1, double y1, double r) {

		if( (x0-x1)*(x0-x1)+(y0-y1)*(y0-y1) < (r*r) ) return true;
		
		return false;

}





// Create mesh of goal
