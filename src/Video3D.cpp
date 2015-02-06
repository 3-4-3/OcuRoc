#include "Video3D.h"
#include <OgreHardwarePixelBuffer.h>
#include <OgreHardwareBuffer.h>

#include <iostream>

Video3D::Video3D(Ogre::Entity *pSnapshot, Ogre::SceneNode *pSceneNode, const Ogre::TexturePtr &depthTexture, const Ogre::TexturePtr &rgbTexture, bool is_left) {
	// basically remember these things for later
	this->snapshot = pSnapshot;
	this->targetSceneNode = pSceneNode;
	///if(is_left)  // carlos
	this->targetSceneNode->setInheritOrientation(false); 
	this->depthTexture = depthTexture;
	this->rgbTexture = rgbTexture;
	this->attached = false;
	// 1st CAMERA (LEFT)
	if(is_left)
	this->snapshot->setMaterialName("roculus3D/DynamicTextureMaterial");
	// 2nd CAMERA (RIGHT)
	else 
	this->snapshot->setMaterialName("roculus3D/DynamicTextureMaterial2");
}

Video3D::~Video3D() {
}

bool Video3D::update(const Ogre::Image &depth, const Ogre::Image &rgb, const Ogre::Vector3 &pos, const Ogre::Quaternion &orientation) {
	// copy the image buffers to the texture buffers (considering the PixelFormat, make sure they match for maximum speed!)
	//std::cout << "RGB POINTER: " << &rgb << std::endl ;
	
	depthTexture->getBuffer()->blitFromMemory(depth.getPixelBox());
	rgbTexture->getBuffer()->blitFromMemory(rgb.getPixelBox());
	
	// update scene node and do some transformation magic
	targetSceneNode->setPosition(pos);
	targetSceneNode->setOrientation(orientation);
	
	//targetSceneNode->roll(Ogre::Degree(-90));
	targetSceneNode->pitch(Ogre::Degree(180));
	
	// attach this node on the first method call
	if (!attached) {
		targetSceneNode->attachObject(snapshot);
		attached = true;
	}
	return true;
}

/* Setters and Getters. Probably unused and therefore redundant.*/

Ogre::SceneNode* Video3D::getTargetSceneNode() {
	return this->targetSceneNode;
}

Ogre::TexturePtr Video3D::getAssignedDepthTexture() {
	return this->depthTexture;
}

Ogre::TexturePtr Video3D::getAssignedRGBTexture() {
	return this->rgbTexture;
}

void Video3D::setTargetSceneNode(Ogre::SceneNode* node) {
	this->targetSceneNode = node;
}

void Video3D::assignDepthTexture(const Ogre::TexturePtr &tex) {
	this->depthTexture = tex;
}

void Video3D::assignRGBTexture(const Ogre::TexturePtr &tex) {
	this->rgbTexture = tex;
}

