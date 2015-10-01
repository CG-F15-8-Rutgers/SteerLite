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
    p0.y = p0.y + .1;

	for(float t=window; t < controlPoints[(controlPoints.size() - 1)].time; t += window){
        calculatePoint(p1, t);
        p1.y = p1.y + .1;

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
    std::vector<CurvePoint>::iterator it;

    /*
    // Debug code
    std::cout << "unsorted" << std::endl;
    for(it=controlPoints.begin() ; it < controlPoints.end(); it++) {
        std::cout << (*it).time << std::endl;
    }
    */

    std::sort(controlPoints.begin(), controlPoints.end(), controlPointsComp);

    /*
    // Debug code
    std::cout << "sorted" << std::endl;
    for(it=controlPoints.begin() ; it < controlPoints.end(); it++) {
        std::cout << (*it).time << std::endl;
    }
    */

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

    for(it=controlPoints.begin() ; it < controlPoints.end(); it++) {
        if(time < (*it).time) {
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

	CurvePoint currentPoint = controlPoints[nextPoint - 1];
	CurvePoint nextPt = controlPoints[nextPoint];
	
	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = nextPt.time - currentPoint.time;
	normalTime = (time - currentPoint.time)/intervalTime;

	// Calculate position at t = time on Hermite curve
	newPosition = (2*normalTime*normalTime*normalTime - 3*normalTime*normalTime + 1)*
		currentPoint.position + (normalTime*normalTime*normalTime - 2*normalTime*normalTime + normalTime)*
		currentPoint.tangent +(-2*normalTime*normalTime*normalTime +3*normalTime*normalTime)*nextPt.position +
		(normalTime*normalTime*normalTime - normalTime*normalTime)*nextPt.tangent;
	// Return result
    //std::cout << newPosition.x << "," << newPosition.y << "," << newPosition.z << std::endl;
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function CatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	CurvePoint currentPoint = controlPoints[nextPoint - 1];
	CurvePoint nextPt = controlPoints[nextPoint];
	CurvePoint lastPt = controlPoints[nextPoint - 2];

	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = nextPt.time - lastPt.time;
	normalTime = (currentPoint.time - time)/intervalTime;

	// Calculate position at t = time on Catmull-Rom curve

	
	// Return result
	return newPosition;
}
