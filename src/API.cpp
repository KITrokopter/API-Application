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
 * Initialize the camera modules
 */
void API::initializeCameras() {
    ros::NodeHandle n;
    
    camera_application::InitializeCameraService message;
    message.request.header.stamp = ros::Time::now();
    
    std::vector<uint32_t> quadcopterIdsVector;
    MapKeysToVec(this->quadcopters, quadcopterIdsVector);
    message.request.quadCopterIds = &quadcopterIdsVector;
    
    uint32_t hsvColorRanges[quadcopterIdsVector.size()];
    for (int i = 0; i < quadcopterIdsVector.size(); i++) {
	hsvColorRanges[2*i] = this->quadcopters.get[i].getColorRange()[0];
	hsvColorRanges[2*i+1] = this->quadcopters.get[i].getColorRange()[1];
    }
    
    std::vector<uint32_t> cameraIds;
    MapKeysToVec(this->cameras, cameraIds);
    
    std::stringstream sstm;
    
    for (int i = 0; i < cameraIds.size(); i++) {
	//empty the stringstream
	sstm.str("");
	sstm << "InitializeCameraService" << this->cameraIds[i];
	//copy the original message to be able to use the response
	camera_application::InitializeCameraService messageCopy = message;
	ros::ServiceClient client = n.serviceClient<camera_application::InitializeCameraService>(sstm.str();
	
	if (client.call(messageCopy) && messageCopy.response.error = 0) {
	    ROS_INFO("Initialized camera %d", this->cameraIds[i])
	} else {
	    ROS_ERROR("Error while initializing camera %d", this->cameraIds[i]);
	}
    }
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

template <typename M, typename V> 
void MapKeysToVec(const  M & m, V & v) {
    for(typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
	v.push_back(it->first);
    }
}

template <typename M, typename V> 
void MapValuesToVec(const  M & m, V & v) {
    for(typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
	v.push_back(it->second);
    }
}