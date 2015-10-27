/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    return false; // There is no collision
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	
}

bool SteerLib::GJK_EPA::EPA(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const std::vector<Util::Vector>& _simplex)
{
	std::vector<Util::Vector> S = _simplex;
	Util::Point origin (0,0,0);
	const float TOLERANCE = 0.000001;
	float distance = 1.0;
	
	while(distance > TOLERANCE)
	{
		float simplex_Distance = Float.MAX_VALUE;
		float temp;
		int temp_i = 0;
		int temp_j = 0;
		for (int i = 0; i < S.getSize(); ++i) {
			int j = (i+1)%S.getSize();
			temp = distSqPointLineSegment(S.at(i),S.at(j),origin);
			if(temp < simplex_Distance){
				simplex_Distance = temp;
				temp_i = i;
				temp_j = j;
			}
		}
		float k = (dot(origin,S.at(temp_j)))/(dot(S.at(temp_j),S.at(temp_j)));
		Util::Point edgepoint = k * S.at(temp_j);
		Util::Point simplexpoint = support(_shapeA, _shapeB, edgepoint);
		S.insert(simplexpoint);
	}
	
}




Util::Point SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const Util::Point _point)
{
	std::vector<Util::Vector> A = _shapeA;
	std::vector<Util::Vector> B = _shapeB;
	Util::Point P = _point;
	Util::Point supportA;
	Util::Point supportB;
	Util::Point support;
	float distA = Float.MAX_VALUE;
	float distB = Float.MAX_VALUE;
	for (i = 0; i < _shapeA.getSize();++i){
		if(_shapeA.at(i)-P < distA){
			distA = _shapeA.at(i) - P;
			supportA = _shapeA.at(i);
		}
	}
	for (i = 0; i < _shapeB.getSize();++i){
		if(_shapeB.at(i)-P < distB){
			distB = P - _shapeB.at(i);
			supportB = _shapeB.at(i);
		}
	}
	support = supportA - supportB;
	return support;
}