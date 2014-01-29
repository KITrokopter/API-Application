#pragma once

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
