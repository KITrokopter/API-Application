#pragma once

#include <vector>
#include <stdint.h>

#include "Vector.hpp"
#include "APIQuadcopterListener.hpp"
#include "ros/ros.h"
#include <ros/console.h>

namespace kitrokopter {

class APIQuadcopter {

	public:
		APIQuadcopter(int id);
		
		int getId();
		
		int[] scanChannels();
		bool connectOnChannel();
		
		uint8_t getChannel();

		Vector getTargetOrientation();
		Vector getTargetPosition();
		Vector getTargetSpeed();
		Vector getTargetAcceleration();
		Vector getCurrentOrientation();
		Vector getCurrentPosition();
		Vector getCurrentSpeed();
		Vector getCurrentAcceleration();

		bool isTracked();

		uint32_t[2] getColorRange();
		float32 getLinkQuality();

		float32 getStabilizerRollData();
		float32 getStabilizerPitchData();
		float32 getStabilizerYawData();

		void setSelectedForFlight(bool select);
		bool isSelectedForFlight();
		
		void setColorRange(uint32_t min, uint32_t max);
		void setColorRange(uint32_t range[2]);

		void blink();

		void addQuadcopterListener(APIQuadcopterListener*);
		void removeQuadcopterListener(APIQuadcopterListener*);
                
                void statusCallback(const quadcopter_application::quadcopter_status::ConstPtr &msg);


	private:
		ros::NodeHandle nodeHandle;
		bool selectedForFlight;
		int id;
		uint8_t channel;
		uint32_t colorRange[2];
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
