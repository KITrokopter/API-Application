#include "APICameraSystem.hpp"

using namespace kitrokopter;

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
