#include "APICameraSystem.hpp"

using namespace kitrokopter;

/**
 * @return A pointer to the camera with the given id.
 */
APICamera APICameraSystem::getCamera(int id)
{
    return cameras.at(id);
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
 * Finds all calibrated cameras.
 *
 * @return A vector of cameras.
 */
std::vector<APICamera> getCalibratedCameras()
{
    std::vector<APICamera> result;
    for (std::vector<APICamera>::iterator it = result.begin(); it != result.end(); ++it) {
	if (it->isCalibrated())
	    result.push_back(*it);
    }
    return result;
}

/**
 * Finds all uncalibrated cameras.
 *
 * @return A vector of cameras.
 */
std::vector<APICamera> getUncalibratedCameras()
{
    std::vector<APICamera> result;
    for (std::vector<APICamera>::iterator it = result.begin(); it != result.end(); ++it) {
	if (!it->isCalibrated())
	    result.push_back(*it);
    }
    return result;
}
