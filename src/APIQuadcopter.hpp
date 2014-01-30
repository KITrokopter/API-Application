#pragma once

#include <vector>

#include "Vector.hpp"
#include "APIQuadcopterListener.hpp"

namespace kitrokopter {

// FIXME: What's a CV_HSV?
typedef int CV_HSV;

class APIQuadcopter {

	public:

		int getId();

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
		int getNetworkLatency();
		int getLinkQuality();
		int getAltimeterAltitude();
		Vector getGyroscopeData();
		Vector getMagnetometerData();
		Vector getAccelerometerData();

		void setSelectedForFlight(bool);
		void setColorRange(CV_HSV first, CV_HSV second);
		void setColorRange(CV_HSV range[2]);

		void blink();

		void addQuadcopterListener(APIQuadcopterListener*);
		void removeQuadcopterListener(APIQuadcopterListener*);


	private:
		bool selectedForFlight;
		int id;
		CV_HSV* colorRange[2];
		int currentSpeed;
		int currentAcceleration;
		Vector currentPosition;
		Vector currentOrientation;
		int latency;
		int linkQuality;
		int altitude;
		Vector gyroscopeData;
		Vector magnetometerData;
		int targetSpeed;
		int targetAcceleration;
		Vector targetPostion;
		Vector targetOrientation;
		Vector accelerometerData;

};

}
