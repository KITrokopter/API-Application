#pragma once

#include <stdint.h>
#include <vector>

#include "Vector.hpp"
#include "APIQuadcopter.hpp"
#include "APIFormation.hpp"
#include "APICamera.hpp"
#include "APICameraSystem.hpp"
#include "APIMessageListener.hpp"

#include "api_application/Announce.h"

namespace kitrokopter {
    
    class API {
	
    public:
	
	API(int argc, char **argv);
	~API();
	
	bool announce(
	    api_application::Announce::Request &req,
	    api_application::Announce::Response &res);
	
	void initializeCameras();
	bool initializeQuadcopters();
	
	/* Quadcopter mutation */
	APIQuadcopter* getQuadcpoter(int id);
	bool removeQuadcopter(int id);
	
	/* Formation */
	void setFormation(APIFormation formation);
	APIFormation* getFormation();
	
	/* Cameras */
	APICameraSystem* getCameraSystem();
	std::vector<APICamera*> getCameras();
	std::vector<APICamera*> getCalibratedCameras();
	std::vector<APICamera*> getUncalibratedCameras();
	int getCameraAmount();
	int getCalibratedCameraAmount();
	int getUncalibratedCameraAmount();
	
	/* Quadcopter getters */
	std::vector<APIQuadcopter*> getQuadcopters();
	std::vector<APIQuadcopter*> getQuadcoptersSelectedForFlight();
	std::vector<APIQuadcopter*> getQuadcoptersNotSelectedForFlight();
        /* TODO: Also selectors for quadcopters currently flying or on the ground? */
	std::vector<APIQuadcopter*> getQuadcoptersTracked();
	std::vector<APIQuadcopter*> getQuadcoptersUntracked();
	std::vector<APIQuadcopter*> getQuadcoptersInFormation();
	std::vector<APIQuadcopter*> getQuadcoptersNotInFormation();
	
        std::vector<uint8_t> scanChannels();
        bool initializeQuadcopers();
	
	/* Quadcopter amount */
	int getQuadcopterAmount();
	int getQuadcoptersFlyingAmount();
	int getQuadcoptersOnGroundAmount();
	int getQuadcoptersTrackedAmount();
	int getQuadcoptersUntrackedAmount();
	int getQuadcoptersInFormationAmount();
	int getQuadcoptersNotInFormationAmount();
	
	/* message listeners */
	void addMessageListener(APIMessageListener*);
	void removeMessageListener(APIMessageListener*);

	/* Launch / Land */
	// TODO
	void launchQuadcopters(int height) {}
	double getLaunchProgress();
	bool quadcoptersLaunched();
	void landQuadcopters();
	
	// TODO
	void shutdownSystem() {}
	
	/* Settings */
	Cuboid getMaximumOperatingArea();
	bool setOperatingArea(Cuboid);
	int getMaximumHorizontalSpeed();
	int getMaximumVerticalSpeed();
	int getMaximumHorizontalAcceleration();
	int getMaximumVerticalAcceleration();
	void setReceiveTargetMovementData(bool);
	void setReceiveActualMovementData(bool);
	void setReceiveQuadcopterState(bool);
	
	/* Movement */
	void moveFormation(Vector);
	void rotateFormation(Vector);
	
    private:
	int idCounter;
	ros::AsyncSpinner *spinner;

	//the ids of modules by category
	std::vector<int> controllerIds;
	std::vector<int> positionIds;

	APICameraSystem cameraSystem;
	std::map <uint32_t, APIQuadcopter> quadcopters;

	APIFormation formation;
    };
}
