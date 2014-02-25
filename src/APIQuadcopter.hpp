#pragma once

#include <vector>
#include <cstdint>

#include "Vector.hpp"
#include "APIQuadcopterListener.hpp"
#include "ros/ros.h"
#include <ros/console.h>

namespace kitrokopter {

// FIXME: What's a CV_HSV?
typedef int CV_HSV;

class APIQuadcopter {

	public:
		APIQuadcopter(int id);
		
		int getId();
		
		bool connectOnChannel();
		
		uint8 getchannel();

		Vector getTargetOrientation();
		Vector getTargetPosition();
		Vector getTargetSpeed();
		Vector getTargetAcceleration();
		Vector getCurrentOrientation();
		Vector getCurrentPosition();
		Vector getCurrentSpeed();
		Vector getCurrentAcceleration();

		bool isTracked();

		std::vector<CV_HSV> getColorRange();
		float32 getLinkQuality();

		float32 getStabilizerRollData();
		float32 getStabilizerPitchData();
		float32 getStabilizerYawData();

		void setSelectedForFlight(bool);
		void setColorRange(CV_HSV first, CV_HSV second);
		void setColorRange(CV_HSV range[2]);

		void blink();

		void addQuadcopterListener(APIQuadcopterListener*);
		void removeQuadcopterListener(APIQuadcopterListener*);


	private:
		ros::NodeHandle nodeHandle;
		bool selectedForFlight;
		int id;
		uint8 channel;
		CV_HSV* colorRange[2];
		int currentSpeed;
		int currentAcceleration;
		Vector currentPosition;
		Vector currentOrientation;
		float32 linkQuality;
		int targetSpeed;
		int targetAcceleration;
		Vector targetPostion;
		Vector targetOrientation;

		float32 stabilizerRollData;
		float32 stabilizerPitchData;
		float32 stabilizerYawData;

};

}
