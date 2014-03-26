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
		APIQuadcopter() {}
		APIQuadcopter(int newid);
		
		int getId();
		
		int* scanChannels() { return NULL; }
		bool connectOnChannel() { return false; }
		
		uint8_t getChannel();

                /* TODO: Where do I get this? */
		Vector getTargetOrientation();
		Vector getTargetPosition();
		Vector getTargetSpeed();
                
		Vector getTargetAcceleration();
		Vector getCurrentOrientation();
		Vector getCurrentPosition();
                
                /* TODO: It seems like there is no topic for this, so the api has to calculate it by itself. */
		float getCurrentSpeed();
		float getCurrentAcceleration();

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

		// TODO
		void addQuadcopterListener(APIQuadcopterListener*) {}
		void removeQuadcopterListener(APIQuadcopterListener*) {}
                
                void statusCallback(const quadcopter_application::quadcopter_status::ConstPtr &msg);


	private:
		ros::NodeHandle nodeHandle;
		bool selectedForFlight;
		int id;
		uint8_t channel;
		uint32_t colorRange[2];
		float currentSpeedValues[2];
                uint32_t currentSpeedTimestamps[2];
		float currentAcceleration;
		Vector currentPositionValues[2];
                uint32_t currentPositionTimestamps[2];
		Vector currentOrientation;
		float linkQuality;
		float targetSpeed;
		float targetAcceleration;
		Vector targetPostion;
		Vector targetOrientation;

		float stabilizerRollData;
		float stabilizerPitchData;
		float stabilizerYawData;

};

}
