#pragma once

#include <stdint.h>

#include "Vector.hpp"

namespace kitrokopter {

class APIFormation {

	public:
                APIFormation() {};
		APIFormation(int newQuadcopterAmount, Vector newQuadcopterPositions);

		int getQuadcopterAmount();
		std::vector<Vector*> getQuadcopterPositions();
		Vector getQuadcopterPosition(int index);

		void setMinimumDistance(uint16_t minimumDistance);
                uint16_t getMinimumDistance();
		void setQuadcopterPositions(std::vector<Vector> positions);


	private:
		uint16_t minimumDistance;
		std::vector<Vector> quadcopterPositions;

};

}
