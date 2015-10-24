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
Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const Util::Vector& d)
{
	// get farthest points for _shapeA and _shapeB
	Util::Vector pointA = getFarthestPoint(_shapeA, d);
	Util::Vector pointB = getFarthestPoint(_shapeB, -d);

	// get Minkwoski difference
	Util::Vector minkDiff = pointA - pointB;

	return minkDiff;
}
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

		printf("<x,y,z> = <%f, %f, %f>\n", x, y, z);
	}

	center.x = x / n;
	center.y = y / n;
	center.z = z / n;

	return center;
}

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

	printf("Farthest Point: <%f, %f, %f>\n", farthest.x, farthest.y, farthest.z);

	return farthest;
}

Util::Vector support(const std::vector <Util::Vector>& _shapeA, const std::vector <Util::Vector>& _shapeB, Util::Vector direction)
{
	// _shapeA --> Get farthest point in direction
	// _shapeB --> Get farthest point in -direction

	Util::Vector farthestA = getFarthestPoint(_shapeA, direction);
	Util::Vector farthestB = getFarthestPoint(_shapeB, -(direction));

	Util::Vector minkowski;
	minkowski.x = farthestA.x - farthestB.x;
	minkowski.y = farthestA.y - farthestB.y;
	minkowski.z = farthestA.z - farthestB.z;

	return minkowski;
}	

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


bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	// Util::Vector d(1, 0, 0);
	// std::vector<Util::Vector>::const_iterator point = _shapeA.begin();
	// float dotproduct = point->x * d.x + point->y * d.y + point->z * d.z;
	// float dp = dot(*point, d);
	// printf("dot product: %f vs %f", dotproduct, dp);

	//PrintPoints(_shapeA, _shapeB);	

	/*
		INITIAL SETUP
	*/
	Util::Vector centerA = CreamyCenter(_shapeA);
	Util::Vector centerB = CreamyCenter(_shapeB);
	Util::Vector direction = centerB - centerA;
	Util::Vector minkdiff = support(_shapeA, _shapeB, direction);

	printf("Minkowski: <%f, %f, %f>\n", minkdiff.x, minkdiff.y, minkdiff.z);

	//printf("Creamy Center A: <%f, %f, %f>\n", centerA.x, centerA.y, centerA.z);
	//printf("Creamy Center B: <%f, %f, %f>", centerB.x, centerB.y, centerB.z);	
	//printf("Direction: <%f, %f, %f>\n", direction.x, direction.y, direction.z);
	return false;
}


//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	GJK(_shapeA, _shapeB);

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
