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

class APICameraSystem;

class APICamera {
	friend class APICameraSystem;

	public:
		APICamera() {}
		APICamera(uint32_t newId);
                APICamera(uint32_t newId, uint64_t newHardwareId);
		~APICamera();

		void listen();
		void initialize(std::vector<APIQuadcopter> quadcopters);

		cv::Mat getImage();
                uint32_t getId();
                uint64_t getHardwareId();
                

		/* Calibration */
		int getCalibrationImageCount();
                
                void addCalibrationImage(cv::Mat image);
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
		void sendPictureSendingActivation(bool active);

		void addCameraListener(APICameraListener*);
		void removeCameraListener(APICameraListener*);

	private:
		void setPosition(double x, double y, double z);

		void handlePicture(const camera_application::Picture::Ptr &msg);

		APICalibrationData *calibration;
		bool calibrated;
		std::vector<cv::Mat> calibrationImages;
                cv::Mat lastImage;
		uint32_t id;
                uint64_t hardwareId;
		static const int VERTICAL_DETECTION_ANGLE;
		static const int HORIZONTAL_DETECTION_ANGLE;

		// Listeners
		std::vector<APIImageListener*> imageListeners;
		std::vector<APICameraListener*> cameraListeners;

		// ROS Subscribers
		ros::Subscriber pictureSubscriber;
                
                ros::Publisher pictureSendingActivationPublisher;

		Vector position;
};

}
