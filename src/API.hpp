#pragma once

#include <stdint.h>
#include <vector>

#include "Vector.hpp"
#include "APIQuadcopter.hpp"
#include "APIFormation.hpp"
#include "APICameraSystem.hpp"

#include "api_application/Announce.h"

namespace kitrokopter {

class APICamera;
class APIMessageListener;

    
    class API {
	
    public:
	enum SystemSignals {
            STARTUP = 1,
            SHUTDOWN
	};
	
	API(int argc, char **argv, bool sync = false);
	~API();
	
	bool announce(
	    api_application::Announce::Request &req,
	    api_application::Announce::Response &res);
	
	void initializeCameras();
	bool initializeQuadcopters();
        void sendQuadcoptersToController();
        
        void startSystem();
        void shutdownSystem();
	
	/* Quadcopter mutation */
	APIQuadcopter* getQuadcopter(int id);
	bool removeQuadcopter(int id);
	
	/* Formation */
	void setFormation(APIFormation formation);
	APIFormation* getFormation();
        void sendFormation();
	
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
	
	/* Settings */
	Cuboid getMaximumOperatingArea();
	
	/* Movement */
	void moveFormation(Vector);
	void rotateFormation();
	
    private:
	int idCounter;
	ros::AsyncSpinner *spinner;
        ros::ServiceServer announceService;
        ros::Publisher systemPublisher;
        ros::Publisher formationPublisher;
        ros::Publisher formationMovementPublisher;

	//the ids of modules by category
	std::vector<int> controllerIds;
	std::vector<int> positionIds;
        
        void sendSystemSignal(uint8_t signal);

	APICameraSystem cameraSystem;
	std::map <uint32_t, APIQuadcopter> quadcopters;

	APIFormation formation;
    };
}
