#include "API.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include <map>

using namespace kitrokopter;

/**
 * Initializes the API which means starting the announcement service.
 * 
 * @param argc remapping arguments from the command line for ros
 * @param argv remapping arguments from the command line for ros
 * @param sync whether to do blocking ROS spinning
 */
API::API(int argc, char **argv, bool sync)
{
    this->idCounter = 0;
    //this->formation = NULL;
    this->controllerIds = std::vector<int>(1);
    this->positionIds = std::vector<int>(5);
    this->cameraSystem = APICameraSystem();
    
    ros::init(argc, argv, "api_server");
    ros::NodeHandle nodeHandle;
    announceService = nodeHandle.advertiseService("announce", &API::announce, this);
    ROS_INFO("Ready to deliver IDs.");
    if (sync) {
       ros::spin();
    } else {
       spinner = new ros::AsyncSpinner(1);
       spinner->start();
    }
}

/**
 * Destructs the API, stopping the ROS spinner.
 */
API::~API()
{
   delete spinner;
}

/**
 * Get a pointer to the APIQuadcopter with the corresponding id 
 * or a null pointer if there is no such APIQuadcopter.
 * 
 * @param id the APIQuadcopter's id
 * @return the APIQuadcopter with this id or a null pointer if there is no such APIQuadcopter
 */
APIQuadcopter* API::getQuadcopter(int id) {
    if (this->quadcopters.find(id) != this->quadcopters.end()) {
	return &this->quadcopters[id];
    } else {
	return NULL;
    }
}



/**
 * Removes the APIQuadcopter with the corresponding id.
 * 
 * @param id the APIQuadcopter's id
 * @return whether there was a APIQuadcopter with this id to remove
 */
bool API::removeQuadcopter(int id) {
    if (this->quadcopters.find(id) != this->quadcopters.end()) {
	quadcopters.erase(id);
	return true;
    } else {
	return false;
    }
}

/**
 * Get all channels where a quadcopter is online.
 *
 * @return array of channels
 */ 
std::vector<uint8_t> API::scanChannels() {
   if (this->quadcopters.size() == 0) {
       throw new std::runtime_error("no quadcopter module to scan with");
   }
   return this->quadcopters.begin()->second.scanChannels();
}

/**
 * Scan for all available quadcopters and distribute them to the quadcopter modules.
 * 
 * @return whether the function was able to distribute all quadcopter channels or or give all quadcopter modules a channel.
 */
bool API::initializeQuadcopters() {
    if (this->quadcopters.size() == 0) {
        return false;
    }
    std::vector<uint8_t> channels = this->scanChannels();
    if (channels.size() == 0){
        return false;
    }
    int max;
    if (this->quadcopters.size() > channels.size()) {
    	max = channels.size();
    } else {
    	max = this->quadcopters.size();
    }
    std::map<uint32_t, APIQuadcopter>::iterator quadIt = this->quadcopters.begin();
    for (int i = 0; i < max; i++)
    {
    	if (quadIt->second.connectOnChannel(channels[i]) == false)
    	{
    		return false;
    	}
    	quadIt++;
    }
}

/**
 * Function to be called when the announce service is invoked.
 */
bool API::announce(api_application::Announce::Request &req, api_application::Announce::Response &res)
{
    res.id = idCounter++;
    switch (req.type) {
	case 0:
	    this->cameraSystem.addCamera(APICamera(res.id));
	    break;
	case 1:
	    this->quadcopters[res.id] = APIQuadcopter(res.id);
	    break;
	case 2:
	    this->controllerIds.push_back(res.id);
	    break;
	case 3:
	    this->positionIds.push_back(res.id);
	    break;
	default:
	    ROS_ERROR("Malformed register attempt!");
	    res.id = -1;
	    return false;
    }
    ROS_INFO("Registered new module with type %d and id %d", req.type, res.id);
    return true;
}

/**
 * Initializes the cameras
 */
void API::initializeCameras() {

}

/**
 * Sets a new formation.
 * 
 * @param newFormation th enew formation
 */
void API::setFormation(APIFormation newFormation) {
    this->formation = newFormation;
    //TODO: Send formation
}



/**
 * Get the formation.
 * 
 * @return the formation
 */
APIFormation* API::getFormation() {
    return &this->formation;
}

APICameraSystem* API::getCameraSystem()
{
   return &cameraSystem;
}

/**
 * Get a vector of all quadcopters.
 * 
 * @return vector of all quadcopters
 */
std::vector<APIQuadcopter*> API::getQuadcopters() {
    std::vector<APIQuadcopter*> result;
    for (std::map<uint32_t, APIQuadcopter>::iterator it =
    			this->quadcopters.begin(); it != this->quadcopters.end(); ++it)
    {
    		result.push_back(&it->second);
    }
    return result;
}

/**
 * Get all quadcopters which are selected to fly in formation.
 * 
 * @return a vector of pointers to the quadcopters
 */
std::vector<APIQuadcopter*> API::getQuadcoptersSelectedForFlight() {
    std::vector<APIQuadcopter*> result;
    for (std::map<uint32_t, APIQuadcopter>::iterator it =
        this->quadcopters.begin(); it != this->quadcopters.end(); ++it)
    {
        if (it->second.isSelectedForFlight())
        {
            result.push_back(&it->second);
        }
    }
    return result;
}

/**
 * Get all quadcopters which are selected to fly in formation.
 * 
 * @return a vector of pointers to the quadcopters
 */
std::vector<APIQuadcopter*> API::getQuadcoptersNotSelectedForFlight() {
    std::vector<APIQuadcopter*> result;
    for (std::map<uint32_t, APIQuadcopter>::iterator it =
        this->quadcopters.begin(); it != this->quadcopters.end(); ++it)
    {
        if (!it->second.isSelectedForFlight())
        {
            result.push_back(&it->second);
        }
    }
    return result;
}

/**
 * Get all quadcopters which are selected to fly in formation.
 * 
 * @return a vector of pointers to the quadcopters
 */
std::vector<APICamera*> API::getCameras() {
    return this->cameraSystem.getCamerasAsVector();
}

