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
 */
API::API(int argc, char **argv)
{
    this->idCounter = 0;
    
    this->cameras = std::vector<int> >(8);
    this->controllers = std::vector<int>(1);
    this->positions = std::vector<int>(5);
    
    ros::init(argc, argv, "api_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("announce", &API::announce, this);
    ROS_INFO("Ready to deliver IDs.");
    ros::spin();
}

/**
 * Get the APIQuadcopter with the corresponding id 
 * or a null pointer if there is no such APIQuadcopter.
 * 
 * @param id the APIQuadcopter's id
 * @return the APIQuadcopter with this id or a null pointer if there is no such APIQuadcopter
 */
APIQuadcopter getQuadcpoter(int id) {
    if (this->quadcopters.find(id) != this->quadcopters.end()) {
	return this->quadcopters[id];
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
bool removeQuadcopter(int id) {
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
int[] scanChannels() {
   return this->quadcopters.begin().scanChannels(); 
}

/**
 * Function to be called when the announce service is invoked.
 */
void API::announce(api_application::Announce::Request &req, api_application::Announce::Response &res)
{
    res.id = idCounter++;
    switch (req.type) {
	case 0:
	    this->cameraIds.push_back(res.id);
	    break;
	case 1:
	    this->quadcopters[res.id] = new APIQuadcopter(res.id);
	    break;
	case 2:
	    this->controllerIds.push_back(res.id);
	    break;
	case 3:
	    this->positionsIds.push_back(res.id);
	    break;
	default:
	    ROS_ERROR("Malformed register attempt!");
	    res.id = -1;
	    return 1;
    }
    ROS_INFO("Registered new module with type %d and id %d", req.type, res.id);
}

/**
 * Sets a new formation.
 */
void API::setFormation(APIFormation newFormation) {
    this->formation = newFormation;
}



/**
 * Get the formation.
 */
APIFormation API::getFormation() {
    return this->formation;
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