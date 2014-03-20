#pragma once

#include <vector>
#include <stdint.h>

#include "Vector.hpp"
#include "APIQuadcopterListener.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include "quadcopter_application/quadcopter_status.h"

namespace kitrokopter {

class APIQuadcopter {

	public:
		APIQuadcopter();
		APIQuadcopter(int newid);
		
		int getId();
		
		int* scanChannels();
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

		uint32_t* getColorRange();
		float getLinkQuality();

		float getStabilizerRollData();
		float getStabilizerPitchData();
		float getStabilizerYawData();

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
		float linkQuality;
		int targetSpeed;
		int targetAcceleration;
		Vector targetPostion;
		Vector targetOrientation;

		float stabilizerRollData;
		float stabilizerPitchData;
		float stabilizerYawData;

};

}
