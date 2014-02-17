#include "APICamera.hpp"

using namespace kitrokopter;

/**
 * Constructs an APICamera.
 */
APICamera::APICamera() : calibrated(false)
{
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
 * Removes an image listener.
 *
 * Remember to free it!
 *
 * @param listener A pointer to the listener.
 */
void APICamera::removeImageListener(APIImageListener *listener)
{
    for (std::vector<APIImageListener*>::iterator it = imageListeners.begin(); it != imageListeners.end(); ++it) {
	if (*it == listener) {
	    imageListeners.erase(it);
	    return;
	}
    }
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

/**
 * Removes an camera listener.
 *
 * Remember to free it!
 *
 * @param listener A pointer to the listener.
 */
void APICamera::removeCameraListener(APICameraListener *listener)
{
    for (std::vector<APICameraListener*>::iterator it = cameraListeners.begin(); it != cameraListeners.end(); ++it) {
	if (*it == listener) {
	    cameraListeners.erase(it);
	    return;
	}
    }
}
