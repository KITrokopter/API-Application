#pragma once

#include <vector>

namespace kitrokopter {

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

		CV_HSV[2] getColorRange();
		int getNetworkLatency();
		int getLinkQuality();
		int getAltimeterAltitude();
		Vector getGyroscopeData();
		Vector getMagnetometerData();
		Vector getAccelerometerData();

		void setSelectedForFlight(bool);
		void setColorRange(CV_HSV first, CV_HSV second);
		void setColorRange(CV_HSV[2] range);

		void blink();

		void addQuadcopterListener(APIQuadcopterListener);
		void removeQuadcopterListener(APIQuadcopterListener);


	private:
		bool selectedForFlight;
		int id;
		CV_HSV* colorRange[2];
		Status status;
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
