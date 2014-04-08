#include "API.hpp"
#include "APICamera.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include <map>

//messages
#include "api_application/System.h"
#include "api_application/SetFormation.h"
#include "api_application/MoveFormation.h"
#include "control_application/SetQuadcopters.h"
#include "control_application/Rotation.h"

using namespace kitrokopter;

/**
 * This is only a dummy formation
 */
APIFormation dummyFormation() {
    std::vector<Vector> positions;
    positions.push_back(Vector(0.0, 0.0, 0.0));
    return APIFormation(positions, 1);
}

/**
 * Initializes the API which means starting the announcement service.
 * 
 * @param argc remapping arguments from the command line for ros
 * @param argv remapping arguments from the command line for ros
 * @param sync whether to do blocking ROS spinning
 */
API::API(int argc, char **argv, bool sync)
{
    //TODO this is only a dummy
    this->formation = dummyFormation();
    
    this->idCounter = 0;
    //this->formation = NULL;
    this->controllerIds = std::vector<int>(1);
    this->positionIds = std::vector<int>(5);
    this->cameraSystem = APICameraSystem();
    
    ros::init(argc, argv, "api_server");
    ros::NodeHandle nodeHandle;
    announceService = nodeHandle.advertiseService("announce", &API::announce, this);
    ROS_INFO("Ready to deliver IDs.");
    
    formationPublisher = nodeHandle.advertise<api_application::SetFormation>("SetFormation", 1);
    ROS_INFO("Ready to send a formation.");
    
    formationMovementPublisher = nodeHandle.advertise<api_application::MoveFormation>("MoveFormation", 1);
    ROS_INFO("Ready to move a formation.");
    
    systemPublisher = nodeHandle.advertise<api_application::System>("System", 1);
    ROS_INFO("Ready to send system signals.");
    
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
 * Start the system
 */
void API::startSystem() {
   this->sendFormation(); 
   this->initializeCameras();
   this->sendSystemSignal(STARTUP);
}

/**
 * Shutdown the system
 */
void API::shutdownSystem() {
    this->sendSystemSignal(SHUTDOWN);
}

/**
 * Send a system signal.
 * 
 * @param signal the signal to send 
 */
void API::sendSystemSignal(uint8_t signal) {
    api_application::System message;
    message.header.stamp = ros::Time::now();
    message.command = signal;
    this->systemPublisher.publish(message);
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
       throw std::runtime_error("no quadcopter module to scan with");
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
    sendQuadcoptersToController();
    return true;
}

/**
 * Initialize the controller
 */
void API::sendQuadcoptersToController() {
    /**
     * since the service does not provide a good way
     * to deal with quadcopters not selected for flight
     * send only "selected for flight" quadcopters
     */
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<control_application::SetQuadcopters>("SetQuadcopters");
    control_application::SetQuadcopters srv;
    srv.request.header.stamp = ros::Time::now();
    std::vector<APIQuadcopter*> result;
    for (std::map<uint32_t, APIQuadcopter>::iterator it =
        this->quadcopters.begin(); it != this->quadcopters.end(); ++it)
    {
        srv.request.quadcopterIds.push_back(it->first);
    }
    srv.request.amount = srv.request.quadcopterIds.size();
    if (!client.call(srv)) {
        ROS_ERROR("Could not call SetQuadcopters service.");
        throw std::runtime_error("Could not call SetQuadcopters service.");
    }
}

/**
 * Initializes the cameras
 */
void API::initializeCameras() {
    this->cameraSystem.initializeCameras(this->quadcopters);
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
	    this->quadcopters[res.id].listen();
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
 * Initializes the controller
 */
void API::initializeController() {
    this->sendFormation();
}

/**
 * Sets a new formation.
 * 
 * @param newFormation th enew formation
 */
void API::setFormation(APIFormation newFormation) {
    this->formation = newFormation;
    sendFormation();
}

/**
 * Send the formation to the controller
 */
void API::sendFormation() {
    api_application::SetFormation msg;
    msg.header.stamp = ros::Time::now();
    msg.distance = this->formation.getMinimumDistance();
    msg.amount = this->formation.getQuadcopterAmount();
    auto positions = this->formation.getQuadcopterPositions();
    for (auto& pos : *positions)
    {
        msg.xPositions.push_back(pos.getX());
        msg.yPositions.push_back(pos.getY());
        msg.zPositions.push_back(pos.getZ());
    }
    this->formationPublisher.publish(msg);
}

/**
 * Get a pointer to the formation.
 * 
 * @return the formation
 */
APIFormation* API::getFormation() {
    return &this->formation;
}

/**
 * Get a pointer to the camera system
 * 
 * @return the pointer
 */
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

/**
 * Move the formation by a vector
 * 
 * @param vector the vector for moving the formation
 */
void API::moveFormation(Vector vector) {
    api_application::MoveFormation message;
    message.header.stamp = ros::Time::now();
    message.xMovement = vector.getX();
    message.yMovement = vector.getY();
    message.zMovement = vector.getZ();
    this->formationMovementPublisher.publish(message);
}

/**
 * Send a rotate formation signal to the controller
 * TODO the controller needs to provide a proper rotate functionality
 *      to make the formation turn by a choosable angle
 */ 
void API::rotateFormation()
{
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<control_application::Rotation>("Rotation");
    control_application::Rotation srv;
    srv.request.header.stamp = ros::Time::now();
    if (!client.call(srv)) {
        ROS_ERROR("Could not call Rotate service.");
        throw std::runtime_error("Could not call Rotate service.");
    }
}
