#pragma once

#include <stdint.h>

#include "Vector.hpp"

namespace kitrokopter {

class APIFormation {

	public:
                APIFormation() {};
		APIFormation(int newQuadcopterAmount, Vector newQuadcopterPositions);

		uint16_t getQuadcopterAmount();
		std::vector<Vector*> getQuadcopterPositions();
		Vector getQuadcopterPosition(int number);

		void setMinimumDistance(uint16_t minimumDistance);
                uint16_t getMinimumDistance();
		void setQuadcopterPositions(Vector position);


	private:
		uint16_t minimumDistance;
		std::vector<Vector> quadcopterPositions;

};

}
