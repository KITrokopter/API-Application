#pragma once

#include <opencv2/core/core.hpp>

#include "Vector.hpp"

namespace kitrokopter {

class APICalibrationData {

	public:

		APICalibrationData(cv::Mat intrinsics, Vector parameters);

		cv::Mat getIntrinsics();
		Vector getParameters();

	private:
		cv::Mat intrinsics;
		Vector parameters;
};

}
