#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

#include "APICalibrationData.hpp"
#include "APIImageListener.hpp"
#include "APICameraListener.hpp"
#include "Cuboid.hpp"
#include "Vector.hpp"

namespace kitrokopter {

class APICamera {

	public:

		APICamera();

		cv::Mat getImage();

		/* Calibration */
		void startCalibration(int imageAmount, int waitingTime);
		int getCalibrationImageCount();
		std::vector<cv::Mat> getAllCalibrationImages();
		cv::Mat getCalibrationImage(int number);
		void setCalibrationData(APICalibrationData data);
		APICalibrationData getCalibrationData();
		bool isCalibrated();
		void deleteCalibration();

		Vector getPosition();
		Vector getOrientation();

		void addImageListener(APIImageListener*);
		void removeImageListener(APIImageListener*);

		void addCameraListener(APICameraListener*);
		void removeCameraListener(APICameraListener*);

	private:
		APICalibrationData *calibration;
		bool calibrated;
		int id;
		static const int VERTICAL_DETECTION_ANGLE;
		static const int HORIZONTAL_DETECTION_ANGLE;

};

}
