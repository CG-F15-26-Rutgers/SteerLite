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
	float maxdot = -1000000000000, dotproduct;
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

/*
	(A x B) x C = B(C dot A) - A(C dot B)
	(AB x A0) x AB = A0(AB dot AB) - AB(AB dot A0)
	lhs = AB dot AB
	rhs = AB dot A0
*/
Util::Vector DoubleCrosser(Util::Vector AB, Util::Vector A0)
{
	float lhs = (AB.x * AB.x) + (AB.y * AB.y) + (AB.z * AB.z);
	float rhs = (AB.x * A0.x) + (AB.y * A0.y) + (AB.z * A0.z);

	Util::Vector ABxA0xAB;
	ABxA0xAB.x = (A0.x * lhs) - (AB.x * rhs);
	ABxA0xAB.y = (A0.y * lhs) - (AB.y * rhs);
	ABxA0xAB.z = (A0.z * lhs) - (AB.z * rhs);

	return ABxA0xAB;
}

bool CheckOrigin(std::vector<Util::Vector>& _simplex, Util::Vector& d)
{
	// Set up
	Util::Vector A, B, C, AB, AC, A0, ABPerp, ACPerp;
	float ABDot, ACDot;
	Util::Vector ORIGIN(0, 0, 0);
	A = _simplex[2];
	B = _simplex[1];
	C = _simplex[0];
	printf("A: <%f, %f, %f>\nB: <%f, %f, %f>\nC: <%f, %f, %f>\n", A.x, A.y, A.z, B.x, B.y, B.z, C.x, C.y, C.z);

	AB = B - A;
	AC = C - A;
	A0 = ORIGIN - A;
	printf("AB: <%f, %f, %f>\nAC: <%f, %f, %f>\n", AB.x, AB.y, AB.z, AC.x, AC.y, AC.z);

	// (AC x AB) x AB = AB(AB dot AC) - AC(AB dot AB)
	// Normal
	// Need to move to separate function
	float ABdotAC = (AB.x * AC.x) + (AB.y * AC.y) + (AB.z * AC.z);
	float ABdotAB = (AB.x * AB.x) + (AB.y * AB.y) + (AB.z * AB.z);
	ABPerp.x = (AB.x * ABdotAC) - (AC.x * ABdotAB);
	ABPerp.y = (AB.y * ABdotAC) - (AC.y * ABdotAB);
	ABPerp.z = (AB.z * ABdotAC) - (AC.z * ABdotAB);
	printf("ABPerp: <%f, %f, %f>\n", ABPerp.x, ABPerp.y, ABPerp.z);

	// Check ABPerp dot product
	ABDot = (ABPerp.x * A0.x) + (ABPerp.y * A0.y) + (ABPerp.z * A0.z);

	// (AB x AC) x AC = AC(AC dot AB) - AB(AC dot AC)
	// Normal
	float ACdotAB = (AC.x * AB.x) + (AC.y * AB.y) + (AC.z * AB.z);
	float ACdotAC = (AC.x * AC.x) + (AC.y * AC.y) + (AC.z * AC.z);
	ACPerp.x = (AC.x * ACdotAB) - (AB.x * ACdotAC);
	ACPerp.y = (AC.y * ACdotAB) - (AB.y * ACdotAC);
	ACPerp.z = (AC.z * ACdotAB) - (AB.z * ACdotAC);
	printf("ACPerp: <%f, %f, %f>\n", ACPerp.x, ACPerp.y, ACPerp.z);

	// Check ACPerp dot product
	ACDot = (ACPerp.x * A0.x) + (ACPerp.y * A0.y) + (ACPerp.z * A0.z);

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

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& _simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	// Init some variables
	Util::Vector centerA, centerB, direction, d, A, B, AB, A0, newPoint;
	Util::Vector ORIGIN(0, 0, 0);
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
	A0 = ORIGIN - A;
	d = DoubleCrosser(AB, A0);
	printf("AB: <%f, %f, %f>\nA0: <%f, %f, %f>\n", AB.x, AB.y, AB.z, A0.x, A0.y, A0.z);
	printf("d: <%f, %f, %f>\n", d.x, d.y, d.z);

	while (true) {
		// Get next point for simplex
		newPoint = support(_shapeA, _shapeB, d);
		printf("newPoint: <%f, %f, %f>\n", newPoint.x, newPoint.y, newPoint.z);

		// Test simplex dot product
		dotproduct = (newPoint.x * d.x) + (newPoint.y * d.y) + (newPoint.z * d.z);
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


//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> _simplex;
	if (GJK(_simplex, _shapeA, _shapeB))
		return true;

	 /*
	 * REHASH OF WHAT'S IN THE HEADER FILE 
	 *
	 * float& return_penetration_depth has to be updated for EPA
	 * Vector& return_penetration_vector has to be updated for EPA
	 * _shapeA and _shapeB
	 * every point p(x,y,z) is vector p(x,y,z)
	 * vector<point> gives list of points in polygon as vector<vector>

	 * pseudocode 
	 * GJK(Poly A, Poly B) {
	 * 		if(A collides with B)
	 * 				return (Simplex, true)
	 * 		else
	 * 				return (NULL, false)
	 * }	
	 *
	 * EPA(Poly A, Poly B, Simplex S) {
	 * 		penetration()
	 * 		return(penetration_depth, penetration_vector)
	 * 
	 * }
	 *
	 * intersect(Poly A, Poly B) {
	 *		(Simplex, isColliding) = GJK(A, B)
	 * 		
	 * 		if(is_colliding == true) {
	 * 				(penetration_depth, penetration_vector) = EPA(A,B,Simplex)
	 *				return(true, penetration_depth, penetration_vector)
	 *		} else {
	 * 				return(false, 0, NULL)
	 * 		}
	 *
	 * 
	 * }
	 */



    return false; // There is no collision
}
