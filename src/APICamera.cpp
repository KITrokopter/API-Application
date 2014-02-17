#include "APICamera.hpp"

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
