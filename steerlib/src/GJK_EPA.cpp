/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

/*
 * Returns True if shapeA and shapeB intersect, False if not.
 * Also sets a value to simplex if shapeA and shapeB intersect
bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
    
}
 */

Util::Vector SteerLib::GJK_EPA::Support(Util::Vector& direction, std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
    Util::Vector maxA = MaxPointAlongDirection(direction, _shapeA);
    Util::Vector maxB = MaxPointAlongDirection(direction * -1, _shapeB);
    return maxA - maxB;
}

Util::Vector SteerLib::GJK_EPA::MaxPointAlongDirection(Util::Vector& direction, const std::vector<Util::Vector>& _shape) {
    float maxDist = -1 * std::numeric_limits<float>::infinity;
    Util::Vector maxPoint;

    for(std::vector<Util::Vector>::iterator it = _shape.begin(); it != _shape.end(); it++) {
        float dot = (*it) * direction;

        if(dot > maxDist) {
            maxDist = dot;
            maxPoint = *it;
        }
    }

    return maxPoint;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    return false; // There is no collision
}
