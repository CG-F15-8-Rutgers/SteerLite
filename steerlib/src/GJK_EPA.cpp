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

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	
}

bool SteerLib::GJK_EPA::EPA(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const std::vector<Util::Vector>& _simplex, float& penetration_depth, Util::Vector& penetration_vector)
{
	std::vector<Util::Vector> S = _simplex;
	Util::Point origin (0,0,0);
	const float TOLERANCE = 0.000001;
	bool done = false;
	
	while(!done)
	{
		float simplex_Distance = std::numeric_limits<float>::max();
		float temp;
		int temp_i;
		int temp_j;
		
		for (int i = 0; i < S.size(); ++i) {
			int j = (i+1)%S.size();
			temp = distSqPointLineSegment(S.at(i),S.at(j),origin);
			if(temp < simplex_Distance){
				simplex_Distance = temp;
				temp_i = i;
				temp_j = j;
			}
		}
		float k = (dot(origin,S.at(temp_j)))/(dot(S.at(temp_j),S.at(temp_j)));
		Util::Vector edgepoint = k * S.at(temp_j);
		Util::Vector simplexpoint = Support(edgepoint, _shapeA, _shapeB);
		
		std::vector<Util::Vector>::iterator it = S.begin();
		for (int temp = 0; temp < temp_i-1; temp++) {
			++it;
		}
		S.insert(it, simplexpoint);
		
		Util::Vector Diff = edgepoint - simplexpoint;
		if ((std::abs(Diff.x) < TOLERANCE) && (std::abs(Diff.y) < TOLERANCE) && (std::abs(Diff.z) < TOLERANCE) && (Diff.length() < TOLERANCE)) {
			done = true;
		}
	}
	
	penetration_depth = MTV.length();
	penetration_vector = MTV;
	return true;
	
}




Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const Util::Vector _direction)
{
	Util::Vector d = _direction;
	Util::Point supportA;
	Util::Point supportB;
	Util::Point support;
	float distA = -1 * std::numeric_limits<float>::max();
	float distB = -1 * std::numeric_limits<float>::max();
	for (i = 0; i < _shapeA.size();++i){
		if((_shapeA.at(i)-d).lengthSquared() > distA){
			distA = (_shapeA.at(i) - d).lengthSquared();
			supportA = _shapeA.at(i);
		}
	}
	for (i = 0; i < _shapeB.size();++i){
		if((_shapeB.at(i)-d).lengthSquared() > distB){
			distB = (d - _shapeB.at(i)).lengthSquared();
			supportB = _shapeB.at(i);
		}
	}
	support = supportA - supportB;
	return support;
}