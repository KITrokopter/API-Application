#include "APICamera.hpp"

#include "ros/ros.h"

// Messages
#include "camera_application/InitializeCameraService.h"
#include "camera_application/PictureSendingActivation.h"
#include "camera_application/CalibrateCamera.h"

#include <string>
#include <stdexcept>

using namespace kitrokopter;

/**
 * Constructs an APICamera.
 */
APICamera::APICamera() : calibrated(false),
			 calibration(NULL)
{
}

/**
 * Destructs an APICamera.
 */
APICamera::~APICamera()
{
    if (calibration)
	delete calibration;
}

void buildInitializationRequest(std::vector<APIQuadcopter> &quadcopters, camera_application::InitializeCameraService &service)
{
    for (std::vector<APIQuadcopter>::iterator it = quadcopters.begin(); it != quadcopters.end();
    ++it) {
        service.request.quadCopterIds.push_back(it->getId());
	std::vector<CV_HSV> colorRange = it->getColorRange();
        service.request.hsvColorRanges.push_back(colorRange.at(0));
        service.request.hsvColorRanges.push_back(colorRange.at(1));
    }
}

/**
 * Initializes the camera with the given quadcopters.
 *
 * Initializing a camera means transmitting quadcopter color ranges.
 *
 * @param quadcopters The quadcopters.
 */
void APICamera::initialize(std::vector<APIQuadcopter> quadcopters)
{
    // Send InitializeCameraService message
    camera_application::InitializeCameraService service;
    buildInitializationRequest(quadcopters, service);

    std::stringstream serviceName;
    std::stringstream err;
    serviceName << "InitializeCameraService" << id;
    if (ros::service::call(serviceName.str(), service)) {
	if (service.response.error != 0) {
	    err << "Service '" << serviceName << "' returned error: " << service.response.error;
	    throw new std::runtime_error(err.str());
	}
    } else {
	err << "Could not call service '" << serviceName << "'.";
	throw new std::runtime_error(err.str());
    }
}

/**
 * Starts the calibration process.
 *
 * @param imageAmount The number of pictures to take.
 * @param waitingTime The time to wait between images in ms.
 */
void APICamera::startCalibration(int imageAmount, int waitingTime, const CalibrationBoard &board)
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<camera_application::CalibrateCamera>("CalibrateCamera", 1);
    camera_application::CalibrateCamera msg;
    msg.ID = this->id;
    msg.imageAmount = imageAmount;
    msg.imageDelay = waitingTime;
    msg.boardWidth = board.width;
    msg.boardHeight = board.height;
    msg.boardRectangleWidth = board.rectangleWidth;
    msg.boardRectangleHeight = board.rectangleHeight;
    pub.publish(msg);
}

/**
 * Sets calibration data.
 *
 * @param data The calibration data to set.
 */
void APICamera::setCalibrationData(APICalibrationData data)
{
    if (calibration)
	*calibration = data;
    else
	calibration = new APICalibrationData(data);
}

/**
 * @return The calibration data.
 */
APICalibrationData* const APICamera::getCalibrationData()
{
    return calibration;
}

/**
 * @return Whether the camera is calibrated.
 */
bool APICamera::isCalibrated()
{
    return calibrated;
}

/**
 * Deletes this camera's calibration.
 */
void APICamera::deleteCalibration()
{
    calibrated = false;
}

/**
 * Adds an image listener.
 *
 * The image listener will receive images this camera takes.
 *
 * @param listener A pointer to the listener.
 */
void APICamera::addImageListener(APIImageListener *listener)
{
    imageListeners.push_back(listener);
}

/**
 * Adds an camera listener.
 *
 * @param listener A pointer to the listener.
 */
void APICamera::addCameraListener(APICameraListener *listener)
{
    cameraListeners.push_back(listener);
}

template <typename T>
void removeListener(std::vector<T*> from, T *listener)
{
    for (typename std::vector<T*>::iterator it = from.begin(); it != from.end(); ++it) {
	if (*it == listener) {
	    from.erase(it);
	    return;
	}
    }
}

/**
 * Removes an image listener.
 *
 * Remember to free it!
 *
 * @param listener A pointer to the listener.
 */
void APICamera::removeImageListener(APIImageListener *listener)
{
    removeListener(imageListeners, listener);
}

/**
 * Removes an camera listener.
 *
 * Remember to free it!
 *
 * @param listener A pointer to the listener.
 */
void APICamera::removeCameraListener(APICameraListener *listener)
{
    removeListener(cameraListeners, listener);
}

/**
 * Sends a message requesting pictures to be or not to be sent.
 *
 * @param active `true` will enable image sending, `false` will disable.
 */
void APICamera::sendPictureSendingActivation(bool active)
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<camera_application::PictureSendingActivation>("PictureSendingActivation", 1);
    camera_application::PictureSendingActivation msg;
    msg.ID = this->id;
    msg.active = active;
    pub.publish(msg);
}
