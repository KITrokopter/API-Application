#include "APIFormation.hpp"

using namespace kitrokopter;

APIFormation::APIFormation() :
    minimumDistance(0),
    quadcopterPositions(std::vector<Vector>())
    {
        
    }

/**
 * Constructor fo a new APIFormation
 * TODO: minDistance seems useless - remove it?
 * 
 * @param quadcopterPositions the positions of the quadcopters
 * @param minDistance a multiplication factor
 */
APIFormation::APIFormation(std::vector<Vector> newQuadcopterPositions, uint16_t newMinDistance)
{
    this->quadcopterPositions = newQuadcopterPositions;
    this->minimumDistance = newMinDistance;
}

/**
 * Get the number of quadcopters needed for this formation
 * 
 * @return the number of quadcopters
 */
int APIFormation::getQuadcopterAmount()
{
    return this->quadcopterPositions.size();
}

/**
 * Get the positions of the quadcopters
 * 
 * @return the quadcopters positions
 */
std::vector<Vector>* APIFormation::getQuadcopterPositions()
{
    return &this->quadcopterPositions;
}

/**
 * Get the position of quadcopter at position [index] in the vector
 * 
 * @param index index of the quadcopter in the vector
 * @return the quadcopter's position
 */
Vector APIFormation::getQuadcopterPosition(int index) {
    return this->quadcopterPositions[index];
}

/**
 * Set the positions
 * 
 * @param positions the positions
 */
void APIFormation::setQuadcopterPositions(std::vector<Vector> positions) {
    this->quadcopterPositions = positions;
}

/**
 * Set the minDistance
 * TODO: minDistance seems useless - remove it?
 * 
 * @param distance the new minDistance
 */
void APIFormation::setMinimumDistance(uint16_t distance)
{
    this->minimumDistance = distance;
}
