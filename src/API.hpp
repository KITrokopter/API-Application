#pragma once

namespace kitrokopter {

class API {

	public:

		API();

		/* Quadcopter mutation */
		APIQuadcopter getQuadcopter(int id);
		void removeQuadcopter(int id);

		/* Formation */
		void setFormation(APIFormation formation);
		APIFormation getFormation();

		/* Cameras */
		APICameraSystem getCameraSystem();
		APICamera[] getCameras();
		APICamera[] getCalibratedCameras();
		APICamera[] getUncalibratedCameras();
		int getCameraAmount();
		int getCalibratedCameraAmount();
		int getUncalibratedCameraAmount();

		/* Quadcopter getters */
		APIQuadcopter[] getQuadcopters();
		APIQuadcopter[] getQuadcoptersFlying();
		APIQuadcopter[] getQuadcoptersOnGround();
		APIQuadcopter[] getQuadcoptersTracked();
		APIQuadcopter[] getQuadcoptersUntracked();
		APIQuadcopter[] getQuadcoptersInFormation();
		APIQuadcopter[] getQuadcoptersNotInFormation();

		/* Quadcopter amount */
		int getQuadcopterAmount();
		int getQuadcoptersFlyingAmount();
		int getQuadcoptersOnGroundAmount();
		int getQuadcoptersTrackedAmount();
		int getQuadcoptersUntrackedAmount();
		int getQuadcoptersInFormationAmount();
		int getQuadcoptersNotInFormationAmount();

		/* message listeners */
		void addMessageListener(APIMessageListener);
		void removeMessageListener(APIMessageListener);

		/* Launch / Land */
		void launchQuadcopters(int height);
		double getLaunchProgress();
		bool quadcoptersLaunched();
		void landQuadcopters();

		void shutdownSystem();

		/* Settings */
		Cuboid getMaximumOperatingArea();
		bool setOperatingArea(Cuboid);
		int getMaximumHorizontalSpeed();
		int getMaximumVerticalSpeed();
		int getMaximumHorizontalAcceleration();
		int getMaximumVerticalAcceleration();
		void setReceiveTargetMovementData(bool);
		void setReceiveActualMovementData(bool);
		void setReceiveQuadcopterState(bool);

		/* Movement */
		void moveFormation(Vector);
		void rotateFormation(Vector);

};

}
