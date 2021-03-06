#include "APICameraSystem.hpp"

// Messages
#include "camera_application/InitializeCameraService.h"
#include "control_application/StartCalibration.h"
#include "control_application/TakeCalibrationPicture.h"
#include "control_application/CalculateCalibration.h"

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
void APICameraSystem::initializeCameras(const std::map<uint32_t, APIQuadcopter> &quadcopters)
{
	camera_application::InitializeCameraService message = this->buildInitMessage(quadcopters);

	std::vector<uint32_t> cameraIds;
	for (std::map<uint32_t, APICamera>::const_iterator it = this->cameras.begin(); it != this->cameras.end(); ++it) {
		cameraIds.push_back(it->first);
	}

	std::stringstream sstm;
	std::stringstream err;

	for (auto id : cameraIds) {
		// empty the stringstream
		sstm.str("");
		sstm << "InitializeCameraService" << id;
		// copy the original message to be able to use the response
		camera_application::InitializeCameraService messageCopy = message;

		if (ros::service::call(sstm.str(), messageCopy)) {
			if (messageCopy.response.error != 0) {
				err << "InitializeCameraService returned error: "
				    << messageCopy.response.error;
				throw std::runtime_error(err.str());
			}
		} else {
			throw std::runtime_error("Could not call InitializeCameraService");
		}
	}
}

/**
 * Create the message to initialize the cameras.
 *
 * @return the message
 */
camera_application::InitializeCameraService APICameraSystem::buildInitMessage(const std::map<uint32_t,
                                                                                             APIQuadcopter> &quadcopters)
{
	camera_application::InitializeCameraService message;
	message.request.header.stamp = ros::Time::now();

	std::vector<uint32_t> quadcopterIds;
	for (std::map<uint32_t, APIQuadcopter>::const_iterator it =
	         quadcopters.begin(); it != quadcopters.end(); ++it) {
		quadcopterIds.push_back(it->first);
	}

	message.request.quadCopterIds = quadcopterIds;

	std::vector<uint32_t> colorRanges;
	for (auto id : quadcopterIds) {
		auto colorRange = quadcopters.at(id).getColorRange();
		colorRanges.push_back(colorRange[0]);
		colorRanges.push_back(colorRange[1]);
	}
	message.request.hsvColorRanges = colorRanges;
	return message;
}

/**
 * Get a pointer to the APIQuadcopter with the corresponding id
 * or a null pointer if there is no such APIQuadcopter.
 *
 * @param id the APIQuadcopter's id
 * @return pointer to the APIQuadcopter with this id or a null pointer if there
 * is no such APIQuadcopter
 */
APICamera* APICameraSystem::getCamera(int id)
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
 * @return A pointer to the map of cameras.
 */
std::map<uint32_t, APICamera>* APICameraSystem::getCamerasAsMap()
{
	return &this->cameras;
}

/**
 * @return A vector of pointers to the cameras.
 */
std::vector<APICamera*> APICameraSystem::getCamerasAsVector()
{
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
void APICameraSystem::addCamera(APICamera camera)
{
	if (this->cameras.find(camera.getId()) != this->cameras.end()) {
		throw std::runtime_error("camera id already in use");
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
std::vector<APICamera*> APICameraSystem::getCalibratedCameras()
{
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

/**
 * Take a picture which will be used for calibration and
 * return the cameras' ids which took a picture wth a chessboard in it
 *
 * @return the ids of the cameras which contain a chessboard
 */
std::map<uint32_t, bool> APICameraSystem::takeCalibrationPictures()
{
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<control_application::TakeCalibrationPicture>(
	    "TakeCalibrationPicture");
	control_application::TakeCalibrationPicture srv;

	if (client.call(srv)) {
		cv::Mat image;
		std::map<uint32_t, bool> result;

		if (srv.response.containsChessboard.size() != srv.response.ids.size()) {
			throw std::runtime_error(
			          "Malformed take calibration picture answer. ContainsChessboard and ids must have the same length!");
		}

		for (unsigned int i = 0; i < srv.response.ids.size(); ++i) {
			result[srv.response.ids[i]] = srv.response.containsChessboard[i];
			auto cam = getCamera(srv.response.ids[i]);

			// Convert uchar[] to cv::Mat manually
			image = cv::Mat(cv::Size(640, 480), CV_8UC3);
			// Don't know if that's necessary. It ensures that the matrix buffer
			// is created.
			// I couldn't find documentation about if the buffer is created by
			// the constructor I used,
			// but when I used similar code I never got a segfault without
			// calling create.
			image.create(cv::Size(640, 480), CV_8UC3);

			for (int j = 0; j < 640 * 480 * 3; j++) {
				image.data[j] = srv.response.images[i].data[j];
			}

			cam->addCalibrationImage(image);
		}

		return result;
	} else {
		ROS_ERROR("Could not call TakeCalibrationPicture.");
		throw std::runtime_error("Could not call the TakeCalibrationPicture ROS service.");
	}
}

/**
 * Send the signal to calculate the calibration after creating the calibration
 * pictures
 * and retrieve the now calculated camera positions.
 */
void APICameraSystem::calculateCalibration()
{
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<control_application::CalculateCalibration>(
	    "CalculateCalibration");
	control_application::CalculateCalibration srv;
	if (client.call(srv)) {
		auto res = srv.response;
		uint32_t id;
		double x, y, z;
		for (unsigned int i = 0; i < res.IDs.size(); ++i) {
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
std::vector<APICamera*> APICameraSystem::getUncalibratedCameras()
{
	std::vector<APICamera*> result;
	for (std::map<uint32_t, APICamera>::iterator it = cameras.begin();
	     it != cameras.end(); ++it) {
		if (it->second.isCalibrated()) {
			result.push_back(&it->second);
		}
	}
	return result;
}

