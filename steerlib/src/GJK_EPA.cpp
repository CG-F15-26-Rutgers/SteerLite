/*!
*
* \author VaHiD AzIzI
*
*/

#include <limits>
#include "obstacles/GJK_EPA.h"

typedef struct Edge{
	float distance;
	Util::Vector normal;
	unsigned int index;
}Edge;


SteerLib::GJK_EPA::GJK_EPA()
{
}

/*
	Gets naive center of a shape
*/
Util::Vector CreamyCenter(const std::vector<Util::Vector>& _shape)
{
	Util::Vector center(0, 0, 0);
	float x, y, z;
	float n = _shape.size();

	for (std::vector<Util::Vector>::const_iterator i = _shape.begin(); i != _shape.end(); ++i) {
		x += i->x;
		y += i->y;
		z += i->z;

		//printf("<x,y,z> = <%f, %f, %f>\n", x, y, z);
	}

	center.x = x / n;
	center.y = y / n;
	center.z = z / n;

	return center;
}

/*
	Gets the farthest point based on a direction
*/
Util::Vector getFarthestPoint(const std::vector<Util::Vector>& _shape, Util::Vector direction)
{
	// it didn't like it when i did maxdot = 0
	float maxdot = -1000, dotproduct;
	Util::Vector farthest;

	for (std::vector<Util::Vector>::const_iterator i = _shape.begin(); i != _shape.end(); ++i) {
		dotproduct = (i->x * direction.x) + (i->y * direction.y) + (i->z * direction.z);
	
		if (dotproduct > maxdot) {
			maxdot = dotproduct;
			farthest = *i;
		}
	}

	//printf("Farthest Point: <%f, %f, %f>\n", farthest.x, farthest.y, farthest.z);

	return farthest;
}

/*
	_shapeA --> get farthest point in direction
	-shapeB --> get farthest point in -direction

	Minkowski difference = farthestA - farthestB 
*/
Util::Vector support(const std::vector <Util::Vector>& _shapeA, const std::vector <Util::Vector>& _shapeB, Util::Vector direction)
{
	Util::Vector farthestA = getFarthestPoint(_shapeA, direction);
	Util::Vector farthestB = getFarthestPoint(_shapeB, -(direction));

	return Util::Vector(farthestA.x - farthestB.x, farthestA.y - farthestB.y, farthestA.z - farthestB.z);
}	

// test function
void PrintPoints(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	printf("A\n");
	for (std::vector<Util::Vector>::const_iterator i = _shapeA.begin(); i != _shapeA.end(); ++i) {
		printf("Point: <%f, %f, %f>\n", i->x, i->y, i->z);
	}

	printf("B\n");
	for (std::vector<Util::Vector>::const_iterator i = _shapeB.begin(); i != _shapeB.end(); ++i) {
		printf("Point: <%f, %f, %f>\n", i->x, i->y, i->z);
	}
}

Util::Vector CrossProduct(Util::Vector u, Util::Vector v){
	// Returns the cross product of two vectors u and v.
	
	Util::Vector uv;
	uv.x = u.y*v.z - u.z*v.y;
	uv.y = u.z*v.x - u.x*v.z;
	uv.z = u.x*v.y - u.y*v.x;
	
	return uv;
}

float DotProduct(Util::Vector u, Util::Vector v){
	// Returns the dot product of two vectors u and v.
	
	return u.x*v.x + u.y*v.y + u.z*v.z;
}

Util::Vector DoubleCrosser(Util::Vector u, Util::Vector v)
{
	// Returns (u x v) x u, which is equivalent to v(u dot u) - u(u dot v)
	
	float uSquared = DotProduct(u, u); 
	float uDotv = DotProduct(u, v);

	return v*uSquared - u*uDotv;
}

bool CheckOrigin(std::vector<Util::Vector>& _simplex, Util::Vector& d)
{
	// Set up
	Util::Vector A, B, C, AB, AC, A0, ABPerp, ACPerp;
	float ABDot, ACDot;

	A = _simplex[2];
	B = _simplex[1];
	C = _simplex[0];
	printf("A: <%f, %f, %f>\nB: <%f, %f, %f>\nC: <%f, %f, %f>\n", A.x, A.y, A.z, B.x, B.y, B.z, C.x, C.y, C.z);

	AB = B - A;
	AC = C - A;
	A0 = -A;
	printf("AB: <%f, %f, %f>\nAC: <%f, %f, %f>\n", AB.x, AB.y, AB.z, AC.x, AC.y, AC.z);

	// (AC x AB) x AB = AB(AB dot AC) - AC(AB dot AB)
	// Normal
	ABPerp = -DoubleCrosser(AB, A0);
	printf("ABPerp: <%f, %f, %f>\n", ABPerp.x, ABPerp.y, ABPerp.z);

	// Check ABPerp dot product
	ABDot = DotProduct(ABPerp, A0);

	// (AB x AC) x AC = AC(AC dot AB) - AB(AC dot AC)
	// Normal
	ACPerp = -DoubleCrosser(AC, AB);
	printf("ACPerp: <%f, %f, %f>\n", ACPerp.x, ACPerp.y, ACPerp.z);

	// Check ACPerp dot product
	ACDot = DotProduct(ACPerp, A0);

	if (ABDot > 0) {
		_simplex.erase(_simplex.begin() + 2);

		d = ABPerp;

		return false;
	}
	else {
		if (ACDot > 0) {
			_simplex.erase(_simplex.begin() + 1);

			d = ACPerp;

			return false;
		}
		else {
			return true;
		}
	}


	return false;
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	// Init some variables
	std::vector<Util::Vector> _simplex;
	Util::Vector centerA, centerB, direction, d, A, B, AB, A0, newPoint;
	float dotproduct;

	// Make sure simplex is clear
	_simplex.clear();
	
	// Check for emptiness then get to the creamy center
	if(!_shapeA.empty())
		centerA = CreamyCenter(_shapeA);
	if(!_shapeB.empty())
		centerB = CreamyCenter(_shapeB);
	
	PrintPoints(_shapeA, _shapeB);
	printf("Creamy Center of A: <%f, %f, %f>\n", centerA.x, centerA.y, centerA.z);
	printf("Creamy Center of B: <%f, %f, %f>\n", centerB.x, centerB.y, centerB.z);

	// Subtract centers to get initial direction
	direction = centerB - centerA;
	printf("Direction: <%f, %f, %f>\n", direction.x, direction.y, direction.z);

	// Get first and second points in simplex
	// Simplex point is found by subtracting the two farthest points from the support function
	// Use the direction based on centers and negate direction for second point
	A = support(_shapeA, _shapeB, direction);
	B = support(_shapeA, _shapeB, -direction);
	printf("A: <%f, %f, %f>\nB:<%f, %f, %f>\n", A.x, A.y, A.z, B.x, B.y, B.z);

	// Add A and B to simplex
	_simplex.push_back(A);
	_simplex.push_back(B);

	// Get new direction
	// (AB x A0) x AB = A0(AB dot AB) - AB(A0 dot AB)
	AB = B - A;
	A0 = -A;
	d = DoubleCrosser(AB, A0);
	printf("AB: <%f, %f, %f>\nA0: <%f, %f, %f>\n", AB.x, AB.y, AB.z, A0.x, A0.y, A0.z);
	printf("d: <%f, %f, %f>\n", d.x, d.y, d.z);

	while (true) {
		// Get next point for simplex
		newPoint = support(_shapeA, _shapeB, d);
		printf("newPoint: <%f, %f, %f>\n", newPoint.x, newPoint.y, newPoint.z);

		// Test simplex dot product
		dotproduct = DotProduct(newPoint, d);
		printf("dotproduct: %f\n", dotproduct);

		if (dotproduct < 0)
			return false;

		// Add newPoint to simplex
		_simplex.push_back(newPoint);

		if (CheckOrigin(_simplex, d))
			return true;
	}

	// "Makes the compiler happy" - Sesh Venugopal
	return false;
}

Edge findClosestEdge(std::vector<Util::Vector> polygon){
	// Returns the edge of the polygon which is closest to the origin.

	Edge closest;
	closest.distance = std::numeric_limits<float>::max();

	for (unsigned int i = 0; i < polygon.size(); ++i){
		unsigned int j = (i+1 == polygon.size()) ? 0 : i + 1;

		Util::Vector a = polygon[i];
		Util::Vector b = polygon[j];
		Util::Vector e = b - a;
		Util::Vector oa = a; // Think of this as a - origin
		Util::Vector n = CrossProduct(CrossProduct(e, oa), e);
		n = Util::normalize(n);

		float d = DotProduct(n, a);

		if (d < closest.distance){
			closest.distance = d;
			closest.normal = n;
			closest.index = j;
		}
	}

	return closest;
}


void SteerLib::GJK_EPA::EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{

	Util::Vector normal;	
	Edge closestEdge;
	
	float epsilon = 0.0001; // Should be a small number.
	
	while (true)
	{
		closestEdge = findClosestEdge(_simplex);
		Util::Vector supportVector = support( _shapeA, _shapeB, closestEdge.normal);

		float d = DotProduct(supportVector, closestEdge.normal);

		if (d - closestEdge.distance < epsilon)
		{
			return_penetration_vector = closestEdge.normal;
			return_penetration_depth = d;
		}

		else
			_simplex.assign(_simplex.begin() + closestEdge.index, supportVector);
	}

}

bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> _simplex;
	float penDepth;
	bool colliding; 
	Util::Vector penVector;

	colliding = GJK(_shapeA, _shapeB);
	if (colliding)
	{
		EPA(return_penetration_depth, return_penetration_vector, _simplex, _shapeA, _shapeB);
		return true;
	}
	else
	{
		penDepth = 0;
		penVector.zero();
		return false;
	}
}

