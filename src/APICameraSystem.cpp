#include "APICameraSystem.hpp"
#include "camera_application/InitializeCameraService.h"
#include "control_application/StartCalibration.h"
#include "control_application/TakeCalibrationPicture.h"
#include "control_application/CalculateCalibration.h"

using namespace kitrokopter;

/**
 * Constructs an APICameraSystem.
 */
APICameraSystem::APICameraSystem() {

}

/**
 * Initialize the camera modules
 * 
 * @param quadcopters The quadcopters which will be tracked.
 */
void APICameraSystem::initializeCameras(
		std::map<uint32_t, APIQuadcopter> quadcopters) {
	camera_application::InitializeCameraService message =
			this->buildInitMessage(quadcopters);

	std::vector < uint32_t > cameraIds;
	for (std::map<uint32_t, APICamera>::const_iterator it =
			this->cameras.begin(); it != this->cameras.end(); ++it) {
		cameraIds.push_back(it->first);
	}

	std::stringstream sstm;
	std::stringstream err;

	for (int i = 0; i < cameraIds.size(); i++) {
		//empty the stringstream
		sstm.str("");
		sstm << "InitializeCameraService" << cameraIds[i];
		//copy the original message to be able to use the response
		camera_application::InitializeCameraService messageCopy = message;

		if (ros::service::call(sstm.str(), messageCopy)) {
			if (messageCopy.response.error != 0) {
				err << "InitializeCameraService returned error: "
						<< messageCopy.response.error;
				throw new std::runtime_error(err.str());
			}
		} else {
			throw new std::runtime_error(
					"Could not call InitializeCameraService");
		}
	}
}

camera_application::InitializeCameraService APICameraSystem::buildInitMessage(
		std::map<uint32_t, APIQuadcopter> quadcopters) {
	camera_application::InitializeCameraService message;
	message.request.header.stamp = ros::Time::now();

	std::vector < uint32_t > quadcopterIds;
	for (std::map<uint32_t, APIQuadcopter>::const_iterator it =
			quadcopters.begin(); it != quadcopters.end(); ++it) {
		quadcopterIds.push_back(it->first);
	}

	message.request.quadCopterIds = quadcopterIds;

	std::vector < uint32_t > colorRanges;
	for (int i = 0; i < quadcopterIds.size(); i++) {
		colorRanges.push_back(quadcopters[i].getColorRange()[0]);
		colorRanges.push_back(quadcopters[i].getColorRange()[1]);
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
APICamera * APICameraSystem::getCamera(int id) {
	if (this->cameras.find(id) != this->cameras.end()) {
		return &cameras[id];
	} else {
		return NULL;
	}
}

/**
 * @return The number of cameras in this system.
 */
int APICameraSystem::getCameraAmount() {
	return cameras.size();
}

/**
 * @return A pointer to the map of cameras.
 */
std::map<uint32_t, APICamera>* APICameraSystem::getCamerasAsMap() {
    return &this->cameras;
}

/**
 * @return A vector of pointers to the cameras.
 */
std::vector<APICamera*> APICameraSystem::getCamerasAsVector() {
    std::vector<APICamera*> result;
    for (std::map<uint32_t, APICamera>::iterator it = cameras.begin();
         it != cameras.end(); ++it) {
        result.push_back(&it->second);
    }
    return result;
}

/**
 * Adds a camera to the camera system.
 * If there is already a camera with this id an exception will be thrown.
 * 
 * @param camera the camera
 */
void APICameraSystem::addCamera(APICamera camera) {
    if (this->cameras.find(camera.getId()) != this->cameras.end()) {
	throw new std::runtime_error("camera id already in use");
    } else {
	uint32_t id = camera.getId();
	this->cameras.insert(
		std::pair<uint32_t, APICamera>(id, camera));
	this->cameras[id].listen();
    }
}

/**
 * Finds all calibrated cameras.
 *
 * @return A vector of pointers to the  cameras.
 */
std::vector<APICamera*> APICameraSystem::getCalibratedCameras() {
	std::vector<APICamera*> result;
	for (std::map<uint32_t, APICamera>::iterator it = cameras.begin();
			it != cameras.end(); ++it) {
		if (it->second.isCalibrated()) {
			result.push_back(&it->second);
		}
	}
	return result;
}

bool APICameraSystem::startCalibration(const CalibrationBoard &board)
{
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<control_application::StartCalibration>("StartCalibration");
    control_application::StartCalibration srv;
    srv.request.chessboardWidth = board.horizontalNumber;
    srv.request.chessboardHeight = board.verticalNumber;
    srv.request.chessboardRealWidth = board.rectangleWidth;
    srv.request.chessboardRealHeight = board.rectangleHeight;
    if (client.call(srv)) {
	return srv.response.ok;
    } else {
	ROS_ERROR("Could not call StartCalibration.");
	return false;
    }
}

int APICameraSystem::takeCalibrationPictures() {
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<control_application::TakeCalibrationPicture>("TakeCalibrationPicture");
    control_application::TakeCalibrationPicture srv;
    if (client.call(srv)) {
	return srv.response.images.size();
    } else {
	ROS_ERROR("Could not call TakeCalibrationPicture.");
	return 0;
    }
}

void APICameraSystem::calculateCalibration()
{
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<control_application::CalculateCalibration>("CalculateCalibration");
    control_application::CalculateCalibration srv;
    if (client.call(srv)) {
	auto res = srv.response;
	uint32_t id;
	double x, y, z;
	for (int i = 0; i < res.IDs.size(); ++i) {
	    id = res.IDs[i];
	    x = res.cameraXPositions[i];
	    y = res.cameraYPositions[i];
	    z = res.cameraZPositions[i];
	    auto cam = getCamera(id);
	    if (cam) {
		cam->setPosition(x, y, z);
	    } else {
		ROS_ERROR("Calibrated invalid camera #%d.", (int)id);
	    }
	}
    } else {
	ROS_ERROR("Could not call CalculateCalibration.");
    }
}

/**
 * Finds all uncalibrated cameras.
 *
 * @return A vector of pointers to the  cameras.
 */
std::vector<APICamera*> APICameraSystem::getUncalibratedCameras() {
	std::vector<APICamera*> result;
	for (std::map<uint32_t, APICamera>::iterator it = cameras.begin();
			it != cameras.end(); ++it) {
		if (it->second.isCalibrated()) {
			result.push_back(&it->second);
		}
	}
	return result;
}
