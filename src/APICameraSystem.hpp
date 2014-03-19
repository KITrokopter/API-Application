#pragma once

#include <vector>
#include <stdint.h>
#include <opencv2/core/core.hpp>

#include "APICamera.hpp"

namespace kitrokopter {

class APICameraSystem {

	public:
                APICameraSystem();
                
		/* Initialization */
                void initializeCameras(std::map<uint32_t, APIQuadcopter> quadcopters);

		/* Camera getters */
		APICamera* getCamera(int id);
		int getCameraAmount();
		std::vector<APICamera> getCameras();
		
		/* Camera setters */
		void addCamera(APICamera camera);

		/* Calibration */
		void startCalibration();
		bool takeCalibrationPicture();
		void calculateCalibration();

		/* Calibration getters */
		bool isCalibrated();
		std::vector<APICamera> getCalibratedCameras();
		std::vector<APICamera> getUncalibratedCameras();
		std::vector< std::vector<cv::Mat> > getCalibrationPictures();
		int getCalibrationPictureCount();

	private:
		std::map <uint32_t, APICamera> cameras;
};

}
