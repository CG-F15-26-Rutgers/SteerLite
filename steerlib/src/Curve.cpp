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
	if (checkRobust()) {

		Point currentPoint = controlPoints[0].position;
		Point nextPoint; // Will be assigned a value in calculatePoint

		float startTime = controlPoints[0].time; // Possibly just set this to 0?
		float endTime = controlPoints[controlPoints.size() - 1].time;

		for (float currentTime = startTime; currentTime <= endTime; currentTime += window) {
			if (calculatePoint(nextPoint, currentTime)) {
				DrawLib::drawLine(currentPoint, nextPoint, curveColor, curveThickness);
				currentPoint = nextPoint;
			}
		}

	}

	return;

#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	//Selection Sort
	for (int i = 0; i < controlPoints.size() - 1; i++)
	{
		//Yo guys!!
		float yyyo = i;
		int j;
		for (j = i + 1; j < controlPoints.size(); j++)
		{
			if (controlPoints[i].time > controlPoints[j].time)
			{
				yyyo = controlPoints[j].time;
			}
		}
		if (yyyo != i)
		{
			float temp = controlPoints.at(i).time;
			controlPoints[i].time = controlPoints[j].time;
			controlPoints[j].time = temp;
		}
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
	if (controlPoints.size() < 2)
		return false;

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	if (controlPoints[0].time == time) {
		nextPoint = 0;

		return true;
	}

	int i;
	for (i = 0; i < controlPoints.size() - 1; i++) {
		if (controlPoints[i].time < time && controlPoints[i + 1].time >= time) {
			nextPoint = i + 1;

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

	if (nextPoint == 0) {
		newPosition = controlPoints[nextPoint].position;

		return newPosition;
	}

	// Calculate time interval, and normal time required for later curve calculations
	int currPoint = nextPoint - 1;
	intervalTime = controlPoints[nextPoint].time - controlPoints[currPoint].time;
	normalTime = (time - controlPoints[currPoint].time) / intervalTime;

	// Calculate position at t = time on Hermite curve
	// Get points
	Point p1 = controlPoints[currPoint].position;
	Point p2 = controlPoints[nextPoint].position;

	// Get tangents
	Vector r1 = controlPoints[currPoint].tangent;
	Vector r2 = controlPoints[nextPoint].tangent;

	// Blending Functions
	float t = normalTime;	// Makes functions easier to read
	float f1 = ((2 * pow(t, 3)) - (3 * pow(t, 2)) + 1);
	float f2 = ((-2 * pow(t, 3)) + (3 * pow(t, 2)));
	float f3 = ((pow(t, 3)) - (2 * pow(t, 2)) + t);
	float f4 = ((pow(t, 4)) - (pow(t, 2)));

	newPosition = (p1 * f1) + (p2 * f2) + (r1 * f3 * intervalTime) + (r2 * f4 * intervalTime);

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime, t1, t2;

	int currPoint = nextPoint - 1;
	int nextNextPoint = nextPoint + 1;
	int prevPoint = currPoint - 1;
	intervalTime = controlPoints[nextPoint].time - controlPoints[currPoint].time;
	normalTime = (time - controlPoints[currPoint].time) / intervalTime;
	t1 = controlPoints[currPoint].time;
	t2 = controlPoints[nextPoint].time;

	Point p1, p2;
	Vector r1, r2;

	if (nextPoint == 1) {
		r1 = (controlPoints[nextPoint].position - controlPoints[currPoint].position) / intervalTime;
		r2 = (controlPoints[nextNextPoint].position - controlPoints[currPoint].position) / (controlPoints[nextNextPoint].time - t1);
	}
	else if (nextPoint == controlPoints.size() - 1) {
		r1 = (controlPoints[nextPoint].position - controlPoints[prevPoint].position) / (t2 - controlPoints[prevPoint].time);
		r2 = (controlPoints[nextPoint].position - controlPoints[currPoint].position) / intervalTime;
	}
	else {
		r1 = (controlPoints[nextPoint].position - controlPoints[prevPoint].position) / (t2 - controlPoints[prevPoint].time);
		r2 = (controlPoints[nextNextPoint].position - controlPoints[currPoint].position) / (controlPoints[nextNextPoint].time - t1);
	}

	float t = normalTime;
	float f1 = (2 * pow(t, 3)) - (3 * pow(t, 2)) + 1;
	float f2 = (-2 * pow(t, 3)) + (3 * pow(t, 2));
	float f3 = (pow(t, 3)) - (2 * pow(t, 2)) + t;
	float f4 = (pow(t, 3)) - (pow(t, 2));

	newPosition = (f1 * controlPoints[currPoint].position) + (f2*controlPoints[nextPoint].position) + (f3*r1*intervalTime) + (f4*r2*intervalTime);


	/*
	if (!nextPoint)
	return controlPoints[0].position;
	else if (nextPoint == controlPoints.size() - 1)
	return controlPoints[controlPoints.size() - 1].position;

	const unsigned int currPoint = nextPoint - 1;
	float normalTime = (time - controlPoints[currPoint].time) / (controlPoints[nextPoint].time - controlPoints[currPoint].time);

	float tSquared = normalTime*normalTime;
	float tCubed = tSquared*normalTime;

	Point P0 = controlPoints[currPoint - 1].position;
	Point P1 = controlPoints[currPoint].position;
	Point P2 = controlPoints[nextPoint].position;
	Point P3 = controlPoints[nextPoint + 1].position;

	Point newPosition;
	newPosition.x = 0.5 * ((3 * P1.x - 3 * P2.x + P3.z - P0.x)*tCubed + (2 * P0.x - 5 * P1.x + 4 * P2.x - P3.x)*tSquared + (P2.x - P0.x)*normalTime + 2 * P1.x);
	newPosition.y = 0.5 * ((3 * P1.y - 3 * P2.y + P3.y - P0.y)*tCubed + (2 * P0.y - 5 * P1.y + 4 * P2.y - P3.y)*tSquared + (P2.y - P0.y)*normalTime + 2 * P1.y);
	newPosition.z = 0.5 * ((3 * P1.z - 3 * P2.z + P3.z - P0.z)*tCubed + (2 * P0.z - 5 * P1.x + 4 * P2.z - P3.z)*tSquared + (P2.z - P0.z)*normalTime + 2 * P1.z);
	*/
	
	return newPosition;
}