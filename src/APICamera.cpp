#include "APICamera.hpp"

using namespace kitrokopter;

APICamera::APICamera() : calibrated(false)
{
}

bool APICamera::isCalibrated()
{
    return calibrated;
}

void APICamera::deleteCalibration()
{
    calibrated = false;
}
