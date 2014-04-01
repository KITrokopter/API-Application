#include "APICamera.hpp"

// Messages
#include "camera_application/InitializeCameraService.h"
#include "camera_application/PictureSendingActivation.h"
#include "camera_application/CalibrateCamera.h"

#include <string>
#include <stdexcept>

using namespace kitrokopter;

/**
 * Constructs an APICamera.
 * 
 * @param newId the cameras id
 */
APICamera::APICamera(uint32_t newId) : calibrated(false),
			 calibration(NULL)
{
    this->id = newId;
}

/**
 * Destructs an APICamera.
 */
APICamera::~APICamera()
{
    if (calibration)
	delete calibration;

    // Delete calibration images.
    for (std::vector<cv::Mat*>::iterator it = calibrationImages.begin(); it != calibrationImages.end();
    ++it) {
        delete *it;
    }
}

uint32_t APICamera::getId()
{
    return this->id;
}

void APICamera::listen()
{
    ros::NodeHandle nh;
    pictureSubscriber = nh.subscribe("Picture", 1, &APICamera::handlePicture, this);
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
    /**
     * TODO: Can we do this now? Is there a hardware id?
     */
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
    
    /**
     * TODO: Is there something to send to the camera?
     */
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

cv::Mat* msgToMat(camera_application::Picture::_image_type data)
{
    /*
     * TODO: Is there a possibility to make the size of the image variable?
     * this hardcoded thing is pretty ugly...
     */
    cv::Mat *mat = new cv::Mat(cv::Size(640, 480), CV_8UC3);
    //640 * 480 * 3 = 921600
    for (size_t i = 0; i < (921600); i++) {
	mat->data[i] = data[i];
    }
    return mat;
}

/**
 * Handles incoming pictures.
 */
void APICamera::handlePicture(const camera_application::Picture::Ptr &msg)
{
    if (msg->ID != this->id) return;
    if (msg->calibrationImage) {
      	cv::Mat* image = msgToMat(msg->image);
        this->lastImage = *image;
        //call the listeners
        for (std::vector<APIImageListener*>::iterator it = imageListeners.begin(); it != imageListeners.end(); ++it) {
            (*it)->imageReceived(*image);
        }
        delete image;
    }
}

Vector APICamera::getPosition()
{
	return position;
}
