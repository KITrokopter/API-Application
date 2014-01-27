#pragma once

#include <vector>

namespace kitrokopter {

class APICameraSystem {

	public:

		/* Camera getters */
		APICamera getCamera(int id);
		int getCameraAmount();
		std::vector<APICamera> getCameras();

		/* Calibration */
		void startCalibration();
		bool takeCalibrationPicture();
		void calculateCalibration();

		/* Calibration getters */
		bool isCalibrated();
		std::vector<APICamera> getCalibratedCameras();
		std::vector<APICamera> getUncalibratedCameras();
		cv::Mat[][] getCalibrationPictures();
		int getCalibrationPictureCount();

	private:
		std::vector<APICamera> cameras;
		int cameraCount;
};

}
