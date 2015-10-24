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

		//printf("<x,y,z> = <%f, %f, %f>\n", x, y, z);
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

	//printf("Farthest Point: <%f, %f, %f>\n", farthest.x, farthest.y, farthest.z);

	return farthest;
}

Util::Vector support(const std::vector <Util::Vector>& _shapeA, const std::vector <Util::Vector>& _shapeB, Util::Vector direction)
{
	// _shapeA --> Get farthest point in direction
	// _shapeB --> Get farthest point in -direction

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

Util::Vector DoubleCrosser(Util::Vector AB, Util::Vector A0)
{
	// (AB x A0) x AB = A0(AB dot AB) - AB(AB dot A0)
	// lhs = AB dot AB
	// rhs = AB dot A0
	float lhs = (AB.x * AB.x) + (AB.y * AB.y) + (AB.z * AB.z);
	float rhs = (AB.x * A0.x) + (AB.y * A0.y) + (AB.z * A0.z);
	Util::Vector ABxA0xAB;
	ABxA0xAB.x = (A0.x * lhs) - (AB.x * rhs);
	ABxA0xAB.y = (A0.y * lhs) - (AB.y * rhs);
	ABxA0xAB.z = (A0.z * lhs) - (AB.z * rhs);

	return ABxA0xAB;
}

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& _simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	//PrintPoints(_shapeA, _shapeB);	

	/*
		INITIAL SETUP
	*/
	Util::Vector centerA;
	Util::Vector centerB;

	if(!_shapeA.empty())
		centerA = CreamyCenter(_shapeA);
	if(!_shapeB.empty())
		centerB = CreamyCenter(_shapeB);
	
	Util::Vector direction = centerB - centerA;
	Util::Vector A = support(_shapeA, _shapeB, direction);
	Util::Vector B = support(_shapeA, _shapeB, -direction);
	
	printf("A = <%f, %f, %f>\nB = <%f, %f, %f>\n", A.x, A.y, A.z, B.x, B.y, B.z);

	Util::Vector AB = (B - A);
	Util::Vector A0 = -A;
	

	//printf("Minkowski 1: <%f, %f, %f>\n", minkdiff.x, minkdiff.y, minkdiff.z);
	//minkdiff = support(_shapeA, _shapeB, -direction);
	//printf("Minkowski 2: <%f, %f, %f>\n", minkdiff.x, minkdiff.y, minkdiff.z);


	//printf("Creamy Center A: <%f, %f, %f>\n", centerA.x, centerA.y, centerA.z);
	//printf("Creamy Center B: <%f, %f, %f>", centerB.x, centerB.y, centerB.z);	
	//printf("Direction: <%f, %f, %f>\n", direction.x, direction.y, direction.z);
	//printf("Minkowski: <%f, %f, %f>\n", minkdiff.x, minkdiff.y, minkdiff.z);
	return false;
}


//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> _simplex;
	GJK(_simplex, _shapeA, _shapeB);

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
