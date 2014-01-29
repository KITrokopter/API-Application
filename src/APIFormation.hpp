#pragma once

namespace kitrokopter {

class APIFormation {

	public:

		APIFormation(int quadcopterAmount, Vector *quadcopterPositions);

		int getQuadcopterAmount();
		Vector* getQuadcopterPositions();
		Vector getQuadcopterPosition(int number);

		void setMinimumDistance(int minimumDistance);
		void setQuadcopterPosition(int number, Vector position);


	private:
		int quadcopterAmount;
		int minimumDistance;
		Vector *quadcopterPositions;

};

}
