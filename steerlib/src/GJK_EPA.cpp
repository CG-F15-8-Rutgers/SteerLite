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
	std::vector<Util::Vector> S;
	bool collision = GJK(_shapeA, _shapeB, S);
	if (collision) {
		EPA(_shapeA, _shapeB, S, return_penetration_depth, return_penetration_vector);
		return collision;
	}
	else {
		return_penetration_depth = 0;
		return_penetration_vector.zero();
		return collision;
	}
    
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex)
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
			_simplex = simplex;
            return true;
        }
    }
}

bool SteerLib::GJK_EPA::EPA(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const std::vector<Util::Vector>& _simplex, float& penetration_depth, Util::Vector& penetration_vector)
{
	std::vector<Util::Vector> S = _simplex;
	Util::Point origin (0,0,0);
	const float TOLERANCE = 0.000001;
	Util::Vector Last;
	Util::Vector Current;
	int count = 0;
	
	for (int i = 0; i < S.size(); ++i) {
		std::cout<<S.at(i).x<<','<<S.at(i).y<<','<<S.at(i).z<<'\n';
	}
	
	while(1)
	{
		float simplex_Distance = std::numeric_limits<float>::max();
		float temp;
		int temp_i;
		int temp_j;
		Last = Current;
		
		for (int i = 0; i < S.size(); ++i) {
			int j = (i+1)%S.size();
			Util::Point tempA (S.at(i).x, S.at(i).y, S.at(i).z);
			Util::Point tempB (S.at(j).x, S.at(j).y, S.at(j).z);
			temp = distSqPointLineSegment(tempA, tempB, origin);
			if(temp < simplex_Distance){
				simplex_Distance = temp;
				temp_i = i;
				temp_j = j;
			}
		}
		float k = (dot((-1*S.at(temp_i)), (S.at(temp_j)-S.at(temp_i))))/(dot((S.at(temp_j)-S.at(temp_i)), (S.at(temp_j)-S.at(temp_i))));
		Util::Vector edgepoint = k*(S.at(temp_j)-S.at(temp_i)) + S.at(temp_i);
		std::cout<<S.at(temp_i).x<<S.at(temp_i).y<<S.at(temp_i).z<<" and ";
		std::cout<<S.at(temp_j).x<<S.at(temp_j).y<<S.at(temp_j).z<<" equals ";
		std::cout<<edgepoint.x<<edgepoint.y<<edgepoint.z<<'\n';
		Util::Vector simplexpoint = Support(edgepoint, _shapeA, _shapeB);
		
		if (temp_i == S.size()-1)
			S.push_back(simplexpoint);
		else {
			std::vector<Util::Vector>::iterator it;
			for (it = S.begin(); it != S.end(); ++it) {
				if ((*it) == S.at(temp_j))
					break;
			}
			//std::cout<<S.at(temp_i).x<<S.at(temp_i).y<<S.at(temp_i).z<<'\n';
			//std::cout<<S.at(temp_j).x<<S.at(temp_j).y<<S.at(temp_j).z<<'\n';
			S.insert(it, simplexpoint);
			//std::cout<<S.at(temp_j).x<<S.at(temp_j).y<<S.at(temp_j).z<<'\n';
		}
		Current = simplexpoint;
		//std::cout<<Current.x<<Current.y<<Current.z<<'\n';
		++count;
		if (count >= 2) {
			Util::Vector Diff = Current - Last;
			if ((std::abs(Diff.x) < TOLERANCE) && (std::abs(Diff.y) <= TOLERANCE) && (std::abs(Diff.z) <= TOLERANCE) &&  (Diff.length() <= TOLERANCE)) {
				penetration_depth = Current.length();
				penetration_vector = Current;
				return true;
			}
		}
	}
	
}




/* Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const Util::Vector _direction)
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
//} */
