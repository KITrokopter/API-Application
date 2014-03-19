#include "APICameraSystem.hpp"

using namespace kitrokopter;

/**
 * Constructs an APICameraSystem.
 */
APICameraSystem::APICameraSystem()
{
    
}

/**
 * Initialize the camera modules
 * 
 * @param quadcopters The quadcopters which will be tracked.
 */
void APICameraSystem::initializeCameras(std::map<uint32_t, APIQuadcopter> quadcopters)
{
    camera_application::InitializeCameraService message = this->buildInitMessage(quadcopters);
    
    std::vector<uint32_t> cameraIds;
    for(std::map<uint32_t, APICamera>::const_iterator it = this->cameras.begin(); it != this->cameras.end(); ++it)
    {
        cameraIds.push_back(it->first);
    }
    
    std::stringstream sstm;
    std::stringstream err;
    
    for (int i = 0; i < cameraIds.size(); i++)
    {
	//empty the stringstream
	sstm.str("");
	sstm << "InitializeCameraService" << cameraIds[i];
	//copy the original message to be able to use the response
	camera_application::InitializeCameraService messageCopy = message;

	if (ros::service::call(sstm.str(), messageCopy))
        {
	    if (messageCopy.response.error != 0)
            {
		err << "InitializeCameraService returned error: " << messageCopy.response.error;
		throw new std::runtime_error(err.str());
	    }
	} else {
            throw new std::runtime_error("Could not call InitializeCameraService");
	}
    }
}

camera_application::InitializeCameraService APICameraSystem::buildInitMessage(std::map<uint32_t, APIQuadcopter> quadcopters)
{
    camera_application::InitializeCameraService message;
    message.request.header.stamp = ros::Time::now();
    
    std::vector<uint32_t> quadcopterIds;
    for(std::map<uint32_t, APIQuadcopter>::const_iterator it = quadcopters.begin(); it != quadcopters.end(); ++it)
    {
        quadcopterIds.push_back(it->first);
    }
    
    message.request.quadCopterIds = quadcopterIds;
    
    std::vector<uint32_t> colorRanges;
    for (int i = 0; i < quadcopterIds.size(); i++) {
        colorRanges.push_back([i].getColorRange()[0]);
        colorRanges.push_back([i].getColorRange()[1]);
    }
    message.request.hsvColorRanges = colorRanges;
    return message;
}

/**
 * Get a pointer to the APIQuadcopter with the corresponding id 
 * or a null pointer if there is no such APIQuadcopter.
 * 
 * @param id the APIQuadcopter's id
 * @return pointer to the APIQuadcopter with this id or a null pointer if there is no such APIQuadcopter
 */
APICamera * APICameraSystem::getCamera(int id)
{
    if (this->cameras.find(id) != this->cameras.end()) {
	return &cameras[id];
    } else {
	return NULL;
    }
}

/**
 * @return The number of cameras in this system.
 */
int APICameraSystem::getCameraAmount()
{
    return cameras.size();
}

/**
 * @return A vector with all cameras.
 */
std::vector<APICamera> APICameraSystem::getCameras()
{
    return cameras;
}

/**
 * Adds a camera to the camera system.
 * If there is already a camera with this id an exception will be thrown.
 * 
 * @param camera the camera
 */
void APICameraSystem::addCamera(APICamera camera)
{
    if (this->cameras.find(camera.getId)) {
	throw new std::runtime_error("camera id already in use");
    } else {
	this->cameras.insert(std::pair<uint32_t,APICamera>(camera.getId,camera));
    }
}

/**
 * Finds all calibrated cameras.
 *
 * @return A vector of pointers to the  cameras.
 */
std::vector<APICamera*> APICameraSystem::getCalibratedCameras()
{
    std::vector<APICamera> result;
    for (std::map<uint32_t,APICamera>::iterator it = cameras.begin(); it != cameras.end(); ++it)
    {
	if (it->isCalibrated())
	{
	    result.push_back(&it->last);
	}
    }
    return result;
}

/**
 * Finds all uncalibrated cameras.
 *
 * @return A vector of pointers to the  cameras.
 */
std::vector<APICamera*> APICameraSystem::getUncalibratedCameras()
{
    std::vector<APICamera> result;
    for (std::map<uint32_t,APICamera>::iterator it = cameras.begin(); it != cameras.end(); ++it)
    {
	if (!it->isCalibrated())
	{
	    result.push_back(&it->last);
	}
    }
    return result;
}