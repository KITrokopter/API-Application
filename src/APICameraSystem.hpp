#pragma once

#include <vector>
#include <stdint.h>
#include <opencv2/core/core.hpp>

#include "APICamera.hpp"

// Messages
#include "camera_application/InitializeCameraService.h"

namespace kitrokopter {

class APICameraSystem {

	public:
                APICameraSystem();
                
		/* Initialization */
                void initializeCameras(std::map<uint32_t, APIQuadcopter> quadcopters);

		/* Camera getters */
		APICamera* getCamera(int id);
		int getCameraAmount();
                std::map <uint32_t, APICamera>* getCameras();
		
		/* Camera setters */
		void addCamera(APICamera camera);

		/* Calibration */
		void startCalibration();
                /*
                 * TODO: How is this to be implemented?
                 * What has to be done by the API?
                 */
		bool takeCalibrationPicture();
		void calculateCalibration();

		/* Calibration getters */
		bool isCalibrated();
		std::vector<APICamera*> getCalibratedCameras();
		std::vector<APICamera*> getUncalibratedCameras();
		std::vector< std::vector<cv::Mat> > getCalibrationPictures();
		int getCalibrationPictureCount();

	private:
		std::map <uint32_t, APICamera> cameras;
                camera_application::InitializeCameraService buildInitMessage(std::map<uint32_t, APIQuadcopter> quadcopters);
};

}
