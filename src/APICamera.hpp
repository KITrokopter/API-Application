#pragma once

#include <vector>
#include <stdint.h>
#include <opencv2/core/core.hpp>

#include "ros/ros.h"

#include "APIQuadcopter.hpp"
#include "APICalibrationData.hpp"
#include "APIImageListener.hpp"
#include "APICameraListener.hpp"
#include "CalibrationBoard.hpp"
#include "Cuboid.hpp"
#include "Vector.hpp"

// Messages
#include "camera_application/Picture.h"

namespace kitrokopter {

class APICamera {

	public:

		APICamera(uint32_t newId;
		~APICamera();

		void initialize(std::vector<APIQuadcopter> quadcopters);

		cv::Mat getImage();

		/* Calibration */
		void startCalibration(int imageAmount, int waitingTime, const CalibrationBoard &board);
		int getCalibrationImageCount();
		std::vector<cv::Mat> getAllCalibrationImages();
		cv::Mat getCalibrationImage(int number);
		void setCalibrationData(APICalibrationData data);
		APICalibrationData* const getCalibrationData();
		bool isCalibrated();
		void deleteCalibration();

		Vector getPosition();
		Vector getOrientation();

		void addImageListener(APIImageListener*);
		void removeImageListener(APIImageListener*);

		void addCameraListener(APICameraListener*);
		void removeCameraListener(APICameraListener*);

	private:

		void sendPictureSendingActivation(bool active);
		void handlePicture(const camera_application::Picture::Ptr &msg);

		APICalibrationData *calibration;
		bool calibrated;
		std::vector<cv::Mat*> calibrationImages;
		uint32_t id;
		static const int VERTICAL_DETECTION_ANGLE;
		static const int HORIZONTAL_DETECTION_ANGLE;

		// Listeners
		std::vector<APIImageListener*> imageListeners;
		std::vector<APICameraListener*> cameraListeners;

		// ROS Subscribers
		ros::Subscriber pictureSubscriber;

};

}
