#pragma once

#include <stdint.h>
#include <vector>

#include "Vector.hpp"

namespace kitrokopter {

class APIFormation {

	public:
                APIFormation();
		APIFormation(std::vector<Vector> newQuadcopterPositions, uint16_t newMinDistance);

		int getQuadcopterAmount();
		std::vector<Vector>* getQuadcopterPositions();
		Vector getQuadcopterPosition(int index);

		void setMinimumDistance(uint16_t minimumDistance);
                uint16_t getMinimumDistance();
		void setQuadcopterPositions(std::vector<Vector> positions);


	private:
		uint16_t minimumDistance;
		std::vector<Vector> quadcopterPositions;

};

}
