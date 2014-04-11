#pragma once

#include <opencv2/core/core.hpp>
#include <stdint.h>

#include "Vector.hpp"

namespace kitrokopter {
class APICalibrationData {
public:

	APICalibrationData(uint32_t newCameraId, uint64_t newCameraHardwareId, bool createdByCamera, double intrinsics[9],
	                   double distortion[4]);
	uint32_t getCameraId;
	uint64_t getCameraHardwareId;
	bool isCreatedByCamera();
	double* getIntrinsics();
	double* getDistortion();

private:
	/*
	 * TODO: If we have the hardware id: do we still need this?
	 */
	uint32_t cameraId;
	/*
	 * TODO: Do we have this yet?
	 */
	uint64_t cameraHardwareId;
	bool createdByCamera;
	double intrinsics[];
	double distortion[];
};
}
