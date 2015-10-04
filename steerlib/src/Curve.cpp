//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
	
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

    // Robustness: make sure there is at least two control point: start and end points
	checkRobust();

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
    Point p0, p1;

    calculatePoint(p0, 0);
    p0.y = p0.y + 0.10;
	//std::cout<<window<<","<<controlPoints[(controlPoints.size()-1)].time<<'\n';

	for(float t= (float)(window); t <= controlPoints[(controlPoints.size() - 1)].time+5; t += (float)(window)){
        calculatePoint(p1, t);
        p1.y = p1.y + 0.10;

        //draw
        DrawLib::glColor(curveColor);
        DrawLib::drawLine(p0, p1);

        p0 = p1;
	}

	return;
#endif
}

bool controlPointsComp(CurvePoint a, CurvePoint b) {
    return a.time < b.time; 
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
    std::vector<CurvePoint>::iterator it = controlPoints.begin();
	++it;
	std::vector<CurvePoint>::iterator it2 = controlPoints.begin();
    
    /* // Debug code
    std::cout << "unsorted" << std::endl;
    for(it=controlPoints.begin() ; it < controlPoints.end(); it++) {
        std::cout << (*it).time << std::endl;
    } */
   

    std::sort(controlPoints.begin(), controlPoints.end(), controlPointsComp);
	//Eliminate control points with identical times until only one remains
	for (it; it < controlPoints.end(); ++it) {
		if ((*it).time == (*it2).time) {
			for (std::vector<CurvePoint>::iterator temp = it; temp < controlPoints.end(); ++temp) {
				if ((*temp).time > (*it).time) {
					Vector dist1 = (*temp).position - (*it2).position;
					Vector dist2 = (*temp).position - (*it).position;
					float mag1 = dist1.x*dist1.x + dist1.y*dist1.y + dist1.z*dist1.z;
					float mag2 = dist2.x*dist2.x + dist2.y*dist2.y + dist2.z*dist2.z;
					if (mag1 > mag2) {
						//delete temp1 = it2
						controlPoints.erase(it2);
					}
					else {
						controlPoints.erase(it);
						it=it2;
					}
					break;
				}
			}
		}
		it2 = it;
	}

    
    // Debug code
    std::cout << "sorted" << std::endl;
    for(it=controlPoints.begin() ; it < controlPoints.end(); it++) {
        std::cout << (*it).time << std::endl;
    }
   

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
    if(controlPoints.size() >= 2) {
        return true;
    } else {
        return false;
    }
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	std::vector<CurvePoint>::iterator it;

	for (it = controlPoints.begin(); it < controlPoints.end(); it++) {
		if (time < (*it).time) {
			nextPoint = std::distance(controlPoints.begin(), it);
			return true;
		}
	}

	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	
	CurvePoint currentPoint = controlPoints[nextPoint-1];
	CurvePoint nextPt = controlPoints[nextPoint];
	
	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = nextPt.time - currentPoint.time;
	normalTime = (time - currentPoint.time)/intervalTime;
	
	// Calculate position at t = time on Hermite curve
	newPosition = (2.0*normalTime*normalTime*normalTime - 3.0*normalTime*normalTime + 1.0)*currentPoint.position
		+ (normalTime*normalTime*normalTime - 2.0*normalTime*normalTime + normalTime)*currentPoint.tangent*intervalTime
		+ (-2.0*normalTime*normalTime*normalTime + 3.0*normalTime*normalTime)*nextPt.position
		+ (normalTime*normalTime*normalTime - normalTime*normalTime)*nextPt.tangent*intervalTime;
	
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	Util::Vector s0, s1;
	CurvePoint currentPt = controlPoints[nextPoint - 1];
	CurvePoint nextPt = controlPoints[nextPoint];

	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = nextPt.time - currentPt.time;
	normalTime = (time - currentPt.time)/intervalTime;

	// Calculate position at t = time on Catmull-Rom curve
	if (nextPoint == 1) {
		//calculate s0 with special case
		CurvePoint otherPt = controlPoints[nextPoint + 1];
		s0 = (otherPt.time - currentPt.time)/(otherPt.time - nextPt.time)*(nextPt.position - currentPt.position)/(nextPt.time-currentPt.time) -
				(nextPt.time - currentPt.time)/(otherPt.time - nextPt.time)*(otherPt.position - currentPt.position)/(otherPt.time - currentPt.time);
	}
	else {
		//calculate as normal
		CurvePoint otherPt = controlPoints[nextPoint - 2];
		s0 = (currentPt.time - otherPt.time)/(nextPt.time - otherPt.time)*(nextPt.position - currentPt.position)/(nextPt.time - currentPt.time) +
				(nextPt.time - currentPt.time)/(nextPt.time - otherPt.time)*(currentPt.position - otherPt.position)/(currentPt.time - otherPt.time);
	}
	if (nextPoint == (controlPoints.size() - 1)) {
		//calculate s1 with special case
		CurvePoint otherPt = controlPoints[nextPoint - 2];
		s1 = (nextPt.time - otherPt.time)/(currentPt.time-otherPt.time)*(nextPt.position - currentPt.position)/(nextPt.time - currentPt.time) -
				(nextPt.time - currentPt.time)/(currentPt.time - otherPt.time)*(nextPt.position - otherPt.position)/(nextPt.time - otherPt.time);
	}
	else {
		//calculate as normal
		CurvePoint otherPt = controlPoints[nextPoint + 1];
		s1 = (nextPt.time - currentPt.time)/(otherPt.time - currentPt.time)*(otherPt.position - nextPt.position)/(otherPt.time - nextPt.time) +
				(otherPt.time - nextPt.time)/(otherPt.time - currentPt.time)*(nextPt.position - currentPt.position)/(nextPt.time - currentPt.time);
	}
	newPosition = (2.0*normalTime*normalTime*normalTime - 3.0*normalTime*normalTime + 1.0)*currentPt.position
		+ (normalTime*normalTime*normalTime - 2.0*normalTime*normalTime + normalTime)*s0*intervalTime
		+ (-2.0*normalTime*normalTime*normalTime + 3.0*normalTime*normalTime)*nextPt.position
		+ (normalTime*normalTime*normalTime - normalTime*normalTime)*s1*intervalTime;

	
	// Return result
	return newPosition;
}
