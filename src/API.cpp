#include "API.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include <map>
#include <vector>

using namespace kitrokopter;

/**
 * Initializes the API which means starting the announcement service.
 * 
 * @param argc remapping arguments from the command line for ros
 * @param argv remapping arguments from the command line for ros
 */
API::API(int argc, char **argv)
{
    this->idCounter = 0;
    //this->formation = NULL;
    this->controllerIds = std::vector<int>(1);
    this->positionIds = std::vector<int>(5);
    this->cameraSystem = APICameraSystem();
    
    ros::init(argc, argv, "api_server");
    this->nodeHandle.advertiseService("announce", &API::announce, this);
    ROS_INFO("Ready to deliver IDs.");
    ros::spin();
}

/**
 * Get a pointer to the APIQuadcopter with the corresponding id 
 * or a null pointer if there is no such APIQuadcopter.
 * 
 * @param id the APIQuadcopter's id
 * @return the APIQuadcopter with this id or a null pointer if there is no such APIQuadcopter
 */
APIQuadcopter* API::getQuadcpoter(int id) {
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
int* API::scanChannels() {
   return this->quadcopters.begin()->second.scanChannels();
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
}



/**
 * Get the formation.
 * 
 * @return the formation
 */
APIFormation API::getFormation() {
    return this->formation;
}

/**
 * Get a vector of all quadcopters.
 * 
 * @return vector of all quadcopters
 */
std::vector<APIQuadcopter> API::getQuadcopters() {
    std:vector<APIQuadcopter> result;
    MapToVec(this->quadcopters, result);
    return result;
}

/**
 * TODO
 */
std::vector<APIQuadcopter> API::getQuadcoptersFlying() {
    
}

/**
 * Add a quadcopter to the API.
 * Throws an exception if there is already a quadcopter with this id.
 * 
 * @param quadcopter th equadcopter to add
 */
void API::addQuadcopter(APIQuadcopter quadcopter) {
    if (this->quadcopters.find(id) == this->quadcopters.end()) {
        throw new std::runtime_error("quadcopter id already in use");
    } else {
        this->quadcopters.insert(std::pair<uint32_t,APIQuadcopter>(quadcopter.getId,quadcopter));
    }
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("quadcopter_status", 1, this->quadcopters[quadcopter.getId].statusCallback);
}




int main(int argc, char** argv)
{
    new API(argc, argv);
    ros::shutdown();
    
    // Wait for ros to shutdown.
    while (ros::ok()) {
	usleep(10000);
    }
    
    std::cout << "API Application successfully terminated" << std::endl;
}
