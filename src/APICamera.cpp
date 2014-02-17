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
