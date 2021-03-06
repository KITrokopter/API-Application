#pragma once

#include <vector>
#include <stdint.h>
#include <vector>

#include "Vector.hpp"
#include "APIQuadcopterListener.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include "quadcopter_application/quadcopter_status.h"
#include "control_application/quadcopter_position.h"
#include "control_application/quadcopter_is_tracked.h"

namespace kitrokopter {
class APIQuadcopter {
public:
	APIQuadcopter()
	{
	}

	APIQuadcopter(int newid);

	void listen();

	int getId();

	std::vector<uint8_t> scanChannels();
	bool connectOnChannel(uint8_t channel);

	int getChannel();

	/* TODO: Where do I get this? */
	Vector getTargetOrientation();
	Vector getTargetPosition();
	Vector getTargetSpeed();

	Vector getTargetAcceleration();
	Vector getCurrentOrientation();
	Vector getCurrentPosition();

	/* TODO: It seems like there is no topic for this, so the api has to
	 * calculate it by itself. */
	Vector getCurrentSpeed();
	Vector getCurrentAcceleration();

	bool isTracked();

	const uint32_t* getColorRange() const;
	float getLinkQuality();

	float getStabilizerRollData();
	float getStabilizerPitchData();
	float getStabilizerYawData();

	float getBatteryStatus();

	void setSelectedForFlight(bool select);
	bool isSelectedForFlight();

	void setColorRange(uint32_t min, uint32_t max);

	void setColorRange(uint32_t range[2]);

	void blink();

	// TODO
	void addQuadcopterListener(APIQuadcopterListener*)
	{
	}

	void removeQuadcopterListener(APIQuadcopterListener*)
	{
	}

	void statusCallback(const quadcopter_application::quadcopter_status::ConstPtr &msg);
	void positionCallback(const control_application::quadcopter_position::ConstPtr &msg);
	void trackingCallback(const control_application::quadcopter_is_tracked::ConstPtr &msg);

private:
	ros::Subscriber statusSubscriber;
	ros::Subscriber positionSubscriber;
	ros::Subscriber trackingSubscriber;

	int id;
	bool selectedForFlight;
	bool trackingStatus;
	int channel;
	uint32_t colorRange[2];
	Vector currentSpeedValues[2];
	double currentSpeedTimestamps[2];
	Vector currentAcceleration;
	Vector currentPositionValues[2];
	double currentPositionTimestamps[2];
	Vector currentOrientation;

	float targetSpeed;
	float targetAcceleration;
	Vector targetPosition;
	Vector targetOrientation;

	float stabilizerRollData;
	float stabilizerPitchData;
	float stabilizerYawData;
	float batteryStatus;
	float linkQuality;

	void updateSpeed();
};
}
