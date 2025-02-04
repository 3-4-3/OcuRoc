/*
-----------------------------------------------------------------------------
Filename:    Roculus.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include <math.h>
#include <simpleXMLparser.h>
#include <simpleSummaryParser.h>
#include "Roculus.h"

typedef pcl::PointXYZRGB PointType;
typedef typename SimpleSummaryParser<PointType>::EntityStruct Entities;
//-------------------------------------------------------------------------------------
Roculus::Roculus(void)
{
}
//-------------------------------------------------------------------------------------
Roculus::~Roculus(void)
{
}

//-------------------------------------------------------------------------------------
void Roculus::createScene(void)
{	
	// Dummy object to generate manual objects and meshes
	Ogre::ManualObject *mPCRender;
	
	// Set camera resolution (X, Y, f) here:
	Ogre::Vector3 cam(640.0f, 480.0f, 574.0f);
	cam = cam/2.0f; // Lower number of vertices (!), comment out for full resolution
	// For better results adapt the resolution parameter in vertexColours.material (!)


	// 1st CAMERA (LEFT)
	// Loading two textures (rgb and depth) for the validity of the standard material and the video stream
	Ogre::TexturePtr pT_RGB = Ogre::TextureManager::getSingleton().createManual(
		"VideoRGBTexture", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_BYTE_RGB,      // pixel format
		Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);  //TU_DYNAMIC_WRITE_ONLY_DISCARDABLE
		
	Ogre::TexturePtr pT_Depth = Ogre::TextureManager::getSingleton().createManual(
		"VideoDepthTexture", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_L16,     		// pixel format
		Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE); 
		
	// 2nd CAMERA (RIGHT)
	// Loading two textures (rgb and depth) for the validity of the standard material and the video stream
	Ogre::TexturePtr pT_RGB2 = Ogre::TextureManager::getSingleton().createManual(
		"VideoRGBTexture2", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_BYTE_RGB,     	// pixel format
		Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);  //TU_DYNAMIC_WRITE_ONLY_DISCARDABLE
		
	Ogre::TexturePtr pT_Depth2 = Ogre::TextureManager::getSingleton().createManual(
		"VideoDepthTexture2", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		640, 480,         		// width & height
		0,                		// number of mipmaps
		Ogre::PF_L16,     		// pixel format
		Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE); 
	
	// Texture to hold the map image
	Ogre::TexturePtr pT_GlobalMap = Ogre::TextureManager::getSingleton().createManual(
		"GlobalMapTexture", 				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		1024, 1024,         	// width & height
		0,                		// number of mipmaps
		Ogre::PF_BYTE_RGBA,     // pixel format
		Ogre::TU_STATIC);
			
	// dummy image to prevent the textures from being dumped at rendering start
	Ogre::Image imDefault;
	imDefault.load("KAMEN320x240.jpg","Popular");
	imDefault.resize(512,512);
	pT_RGB->loadImage(imDefault);
	pT_Depth->loadImage(imDefault);
	pT_RGB2->loadImage(imDefault);
	pT_Depth2->loadImage(imDefault);
	pT_GlobalMap->loadImage(imDefault);

	// 1st CAMERA (LEFT)
	// create the standard geometry for the Snapshots, taking into account the hardcoded 'cam' parameters above
	// better solution: write a cfg file and corresponding parser (see GameCFGParser.h for example)
	mPCRender= mSceneMgr->createManualObject();
	mPCRender->estimateVertexCount(cam.x * cam.y);
	mPCRender->estimateIndexCount(4 * cam.x * cam.y);
	mPCRender->begin("roculus3D/DynamicTextureMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
		float fake_z = 0.0f;
		for (int w=0; w<cam.x; w++) {
			for (int h=0; h<cam.y; h++) {
				if (fake_z > -4.0f) fake_z -= 0.05f;
				mPCRender->position(float(w - (cam.x-1.0f)/2.0f)/cam.z, float((cam.y-1.0f)/2.0f - h)/cam.z, fake_z);
				mPCRender->textureCoord(float(w)/(cam.x-1.0f),float(h)/(cam.y-1.0f));
				if (w>0 && h>0) {
					mPCRender->quad(w*cam.y+h, (w-1)*cam.y+h, (w-1)*cam.y+h-1, w*cam.y+h-1);
					mPCRender->quad(w*cam.y+h, w*cam.y+h-1, (w-1)*cam.y+h-1, (w-1)*cam.y+h);
				}
			}
		}
	mPCRender->end();
	mPCRender->convertToMesh("CamGeometry");
	
	// 2nd CAMERA (RIGHT)
	// create the standard geometry for the Snapshots, taking into account the hardcoded 'cam' parameters above
	// better solution: write a cfg file and corresponding parser (see GameCFGParser.h for example)
	mPCRender= mSceneMgr->createManualObject();
	mPCRender->estimateVertexCount(cam.x * cam.y);
	mPCRender->estimateIndexCount(4 * cam.x * cam.y);
	mPCRender->begin("roculus3D/DynamicTextureMaterial2", Ogre::RenderOperation::OT_TRIANGLE_LIST);
		fake_z = 0.0f;
		for (int w=0; w<cam.x; w++) {
			for (int h=0; h<cam.y; h++) {
				if (fake_z > -4.0f) fake_z -= 0.05f;
				mPCRender->position(float(w - (cam.x-1.0f)/2.0f)/cam.z, float((cam.y-1.0f)/2.0f - h)/cam.z, fake_z);
				mPCRender->textureCoord(float(w)/(cam.x-1.0f),float(h)/(cam.y-1.0f));
				if (w>0 && h>0) {
					mPCRender->quad(w*cam.y+h, (w-1)*cam.y+h, (w-1)*cam.y+h-1, w*cam.y+h-1);
					mPCRender->quad(w*cam.y+h, w*cam.y+h-1, (w-1)*cam.y+h-1, (w-1)*cam.y+h);
				}
			}
		}
	mPCRender->end();
	mPCRender->convertToMesh("CamGeometry");
	
	// create a simple coordinate system for debugging
	mPCRender= mSceneMgr->createManualObject();
	mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_LINE_LIST);
		mPCRender->position(0.0f,0.0f,0.0f);
		mPCRender->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
		mPCRender->position(1.0f,0.0f,0.0f);
		mPCRender->colour(Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
		mPCRender->position(0.0f,1.0f,0.0f);
		mPCRender->colour(Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f));
		mPCRender->position(0.0f,0.0f,1.0f);
		mPCRender->colour(Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f));

		mPCRender->index(0);
		mPCRender->index(1);
		mPCRender->index(0);
		mPCRender->index(2);
		mPCRender->index(0);
		mPCRender->index(3);
	mPCRender->end();
	mPCRender->convertToMesh("CoordSystem");
	
	
	// create an arrow that indicates next objective
	mPCRender= mSceneMgr->createManualObject();
	mPCRender->begin("roculus3D/BlankMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
		float const radius = 0.005;
		float const accuracy = 35; 
		float const max_altitude = 0.6;
		float const min_altitude = 0.5;
		
		float const radius2 = 0.02;
		float const pyramid_base_altitude = 0.53;
		float const corner_altitude = 0.46;
		
		// CILINDER
		unsigned point_index = 0;
		for(float theta = 0; theta <= 2 * Math::PI; theta += Math::PI / accuracy) {
			mPCRender->position(radius * cos(theta), min_altitude, radius * sin(theta));
			mPCRender->colour(Ogre::ColourValue(1.0f, 0.5f, 0.0f, 0.2f));
			
			mPCRender->position(radius * cos(theta), max_altitude, radius * sin(theta));
			mPCRender->colour(Ogre::ColourValue(1.0f, 0.5f, 0.0f, 0.2f));
			
			point_index++;
		}
		
		// PYRAMID
		double pyramid_index = 0;
		for(float theta = 0; theta <= 2 * Math::PI; theta += Math::PI / accuracy) {
			mPCRender->position(radius2 * cos(theta), pyramid_base_altitude, radius2 * sin(theta));
			mPCRender->colour(Ogre::ColourValue(1.0f, 0.5f, 0.0f, 0.2f));
			
			pyramid_index++;
		}
		
		double corner_index = point_index*2+pyramid_index;
		mPCRender->position(0.0f, corner_altitude, 0.0f);
		mPCRender->colour(Ogre::ColourValue(1.0f, 0.5f, 0.0f, 0.2f));
		
		while(pyramid_index>0) {
			if(pyramid_index!=1) {
			mPCRender->triangle(corner_index,point_index*2+pyramid_index-1,point_index*2+pyramid_index-2);
			mPCRender->triangle(point_index*2+pyramid_index-1,corner_index,point_index*2+pyramid_index-2);
			} else {
			mPCRender->triangle(corner_index,point_index*2+pyramid_index-1,corner_index-1);
			mPCRender->triangle(point_index*2+pyramid_index-1,corner_index,corner_index-1);
			}
			
			pyramid_index--;
		}
		
		
		//CILINDER
		mPCRender->triangle(0,1,point_index*2-1);
		mPCRender->triangle(0,point_index*2-1,point_index*2-2);
		
		mPCRender->triangle(1,0,point_index*2-1);
		mPCRender->triangle(point_index*2-1,0,point_index*2-2);
		
		point_index--;
		while(point_index>0) {
			mPCRender->triangle(point_index*2+1,point_index*2,point_index*2-1);
			mPCRender->triangle(point_index*2,point_index*2+1,point_index*2-1);
			
			mPCRender->triangle(point_index*2-2,point_index*2,point_index*2-1);
			mPCRender->triangle(point_index*2-1,point_index*2,point_index*2-2);
			point_index--;
		}
		
	mPCRender->end();
	mPCRender->convertToMesh("Objective");
	

	// construct an instance of the cursor with circle and transparent flagg
	cursor = mSceneMgr->getRootSceneNode()->createChildSceneNode("CircleCursor");
	///cursor->attachObject(mSceneMgr->createEntity("CircleCursor"));
	Ogre::Entity *wpMarker = mSceneMgr->createEntity("Cylinder.mesh");
	wpMarker->setMaterialName("roculus3D/WayPointMarkerTransparent");
	///cursor->attachObject(wpMarker);
	
	// set up the node for the video stream
	// The right video node is child of the left
	Ogre::SceneNode *pSceneNodeL = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	vdVideoLeft = new Video3D(mSceneMgr->createEntity("CamGeometry"), pSceneNodeL, pT_Depth, pT_RGB, true);	
	
	
	///vdVideoRight = new Video3D(mSceneMgr->createEntity("CamGeometry"), pSceneNodeL->createChildSceneNode(), pT_Depth2, pT_RGB2, false);
	
	// creating Robot for pose
	robotModel = new Robot(mSceneMgr, pSceneNodeL->createChildSceneNode());	
	
	///	vdVideoLeft = new Video3D(mSceneMgr->createEntity("CamGeometry"), mSceneMgr->getRootSceneNode()->createChildSceneNode(), pT_Depth, pT_RGB, true);	
	  vdVideoRight = new Video3D(mSceneMgr->createEntity("CamGeometry"), mSceneMgr->getRootSceneNode()->createChildSceneNode(), pT_Depth2, pT_RGB2, false);
	
	/* Good for debugging: add some coordinate systems */
	 ///vdVideoLeft->getTargetSceneNode()->attachObject(mSceneMgr->createEntity("CoordSystem"));
	 ///vdVideoRight->getTargetSceneNode()->attachObject(mSceneMgr->createEntity("CoordSystem"));
	 ///mSceneMgr->getRootSceneNode()->attachObject(mSceneMgr->createEntity("CoordSystem"));
	
	// PREallocate and manage memory to load/record snapshots
	snLib = new SnapshotLibrary(mSceneMgr, Ogre::String("CamGeometry"), Ogre::String("roculus3D/DynamicTextureMaterialSepia"), 10);
    rsLib = new SnapshotLibrary(mSceneMgr, Ogre::String("CamGeometry"), Ogre::String("roculus3D/DynamicTextureMaterialSepia"), 10);
    
    // Load the prerecorded environment    
	loadRecordedScene();
    
}

void Roculus::loadRecordedScene() {
	
	// Rares' parser for the room recordings in a directory
	// the parser was slightly modified to work on a subset of images (see indexing below)
	SimpleSummaryParser<PointType> summary_parser("./map/index.xml");
    summary_parser.createSummaryXML("./map/");
    
	SimpleXMLParser<PointType> parser;
    SimpleXMLParser<PointType>::RoomData roomData;
    std::vector<Entities> allSweeps = summary_parser.getRooms();

	// there is lots of overlap so a subset is actually sufficient:
	// store all indecies that shall be processed and displayed {you can specify higher indices that don't actually exist!}
	std::set<size_t> idc2process; 
	idc2process.insert(0); 	idc2process.insert(2);	idc2process.insert(4);	idc2process.insert(6);	idc2process.insert(8);	idc2process.insert(10);
	idc2process.insert(12);	idc2process.insert(14);	idc2process.insert(16);	

    // Rares: Added for complete semantic map
//    idc2process.insert(18);	idc2process.insert(20);	idc2process.insert(22);idc2process.insert(24);

//	idc2process.insert(34); idc2process.insert(36);	idc2process.insert(38);	idc2process.insert(40);	idc2process.insert(42);	idc2process.insert(44);
//	idc2process.insert(46); idc2process.insert(48);	idc2process.insert(50);
//	idc2process.insert(68);	idc2process.insert(70);	idc2process.insert(72);	idc2process.insert(74);	idc2process.insert(76);	idc2process.insert(78);
//	idc2process.insert(80); idc2process.insert(82);	idc2process.insert(84);
	
	// process the files and insert the Snapshots
	Ogre::Image oi_rgb, oi_depth;
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	tfScalar yaw,pitch,roll;
	Matrix3 mRot;
    for (size_t i=0; i<allSweeps.size(); i++) {
		// load each room that was parsed
		roomData = parser.loadRoomFromXML(allSweeps[i].roomXmlFile, &idc2process);
		for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
		{
			const cv::Mat& rgbImg = roomData.vIntermediateRGBImages[i];
			const cv::Mat& depthImg = roomData.vIntermediateDepthImages[i];
					
			// IMAGE FILTERING (smoothing the depth image):
			cv::GaussianBlur(depthImg, depthImg, cv::Size(11,11), 0, 0);
			
			// load the images into the scene
			oi_rgb.loadDynamicImage(static_cast<uchar*>(rgbImg.data), rgbImg.cols, rgbImg.rows, 1, Ogre::PF_BYTE_RGB);
			oi_depth.loadDynamicImage(static_cast<uchar*>(depthImg.data), rgbImg.cols, rgbImg.rows, 1, Ogre::PF_L16);
			
			// load the corresponding camera transform
            tf::StampedTransform tfCurrent = roomData.vIntermediateRoomCloudTransforms[i];

//            tfCurrent.setIdentity();

            /**************Apply 0.02*M_PI rotation around the Z axis ***********************/
            tf::StampedTransform yaw_rot;
            yaw_rot.setIdentity();
            Eigen::Quaternionf quat_rot(Eigen::AngleAxisf(-0.02*M_PI,Eigen::Vector3f::UnitZ()));
            tf::Quaternion aQuat(quat_rot.x(),quat_rot.y(),quat_rot.z(),quat_rot.w());
            yaw_rot.setRotation(aQuat);
//            tfCurrent*=yaw_rot;
            /**************Apply 0.02*M_PI rotation around the Z axis ***********************/

			position.x = -tfCurrent.getOrigin().y();
			position.y = tfCurrent.getOrigin().z();
			position.z = -tfCurrent.getOrigin().x();

            tf::Matrix3x3 basis = tfCurrent.getBasis();
            tf::Matrix3x3 quaternion_rotation_matrix(aQuat);
            basis*=quaternion_rotation_matrix;

            tfCurrent.getBasis().getEulerYPR(yaw,pitch,roll);
//            basis.getEulerYPR(yaw,pitch,roll);
//            yaw+=0.02 * M_PI; // Rares: NILS Magick
			mRot.FromEulerAnglesXYZ(-Radian(pitch),Radian(yaw),-Radian(roll));
			orientation.FromRotationMatrix(mRot);
			
			// place the object in the scene
			rsLib->placeInScene(oi_depth, oi_rgb, position, orientation);
		}
	}
}

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        Roculus app;

        try {
	  app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
