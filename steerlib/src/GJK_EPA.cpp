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

Util::Vector SteerLib::GJK_EPA::Support(Util::Vector& direction, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
    Util::Vector negD = direction * -1;
    Util::Vector maxA = MaxPointAlongDirection(direction, _shapeA);
    Util::Vector maxB = MaxPointAlongDirection(negD, _shapeB);
    return maxA - maxB;
}

Util::Vector SteerLib::GJK_EPA::MaxPointAlongDirection(Util::Vector& direction, const std::vector<Util::Vector>& _shape) {
    float maxDist = -1 * INFINITY;
    Util::Vector maxPoint;

    for(std::vector<Util::Vector>::const_iterator it = _shape.begin(); it != _shape.end(); it++) {
        float dot = (*it) * direction;

        if(dot > maxDist) {
            maxDist = dot;
            maxPoint = *it;
        }
    }

    return maxPoint;
}

bool SteerLib::GJK_EPA::NearestSimplex(Util::Vector& direction, std::vector<Util::Vector>& simplex) {
    int size = simplex.size();
    Util::Vector origin = Util::Vector();

    if(size == 1) {
        // Should never happen but just return the point in case
        std::cout << "something fucked up. size 1" << std::endl;
        direction = direction * -1;
    } else if(size == 2) {
        Util::Vector A = simplex[1];
        Util::Vector B = simplex[0];

        Util::Vector AB = B - A;
        Util::Vector A0 = origin - A;

        if(AB*A0 > 0) {
            //s = [A, B], d = AB*A0*AB
            direction = cross(cross(AB, A0), AB);

            if(direction.x == 0 && direction.y == 0 && direction.z == 0) {
                // std::cout << "zero dir found" << std::endl;
            }
        } else {
            //s = [A], d = A0
            simplex.erase(simplex.begin());
            direction = A0;
        }

        return false;
    } else if(size == 3) {
        Util::Vector A = simplex[2];
        Util::Vector B = simplex[1];
        Util::Vector C = simplex[0];

        Util::Vector A0 = origin - A;
        Util::Vector AB = B - A;
        Util::Vector AC = C - A;
        Util::Vector ABC = cross(AB, AC);

        if(cross(ABC, AC) * A0 > 0) {
            if(AC * A0 > 0) {
                simplex.erase(simplex.begin() + 1);
                direction = cross(cross(AC, A0), AC);
                return false;
            } else {
                if(AB * A0 > 0) {
                    simplex.erase(simplex.begin());
                    direction = cross(cross(AB, A0), AB);
                    return false;
                } else {
                    //make simplex = A
                    simplex.erase(simplex.begin());
                    simplex.erase(simplex.begin());
                    direction = A0;
                    return false;
                }
            }
        } else {
            if(cross(AB, ABC) * A0 > 0) {
                if(AB * A0 > 0) {
                    simplex.erase(simplex.begin());
                    direction = cross(cross(AB, A0), AB);
                    return false;
                } else {
                    //make simplex = A
                    simplex.erase(simplex.begin());
                    simplex.erase(simplex.begin());

                    direction = A0;
                    return false;
                } 
            } else {
                //origin found
                return true;
            }
        }
    } else {
        //should never happen
        std::cout << "something fucked up. size > 3" << std::endl;
    }
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    return GJK(_shapeA, _shapeB); // There is no collision
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    Util::Vector firstSimplex = _shapeA[0] - _shapeB[0];
    std::vector<Util::Vector> simplex = std::vector<Util::Vector>();
    simplex.push_back(firstSimplex);
    
    Util::Vector direction = firstSimplex * -1;

    while(1) {
        Util::Vector nextPoint = Support(direction, _shapeA, _shapeB);
        if(nextPoint * direction < 0) {
            return false; 
        }

        simplex.push_back(nextPoint);
        if(NearestSimplex(direction, simplex)) {
            return true;
        }
    }
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
  /*
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
    */
}
