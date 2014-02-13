#include "APICameraSystem.hpp"

using namespace kitrokopter;

std::vector<APICamera> getCalibratedCameras()
{
    std::vector<APICamera> result;
    for (std::vector<APICamera>::iterator it = result.begin(); it != result.end(); ++it) {
	if (it->isCalibrated())
	    result.push_back(*it);
    }
    return result;
}

std::vector<APICamera> getUncalibratedCameras()
{
    std::vector<APICamera> result;
    for (std::vector<APICamera>::iterator it = result.begin(); it != result.end(); ++it) {
	if (!it->isCalibrated())
	    result.push_back(*it);
    }
    return result;
}
