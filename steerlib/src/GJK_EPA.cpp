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
bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	Util::Vector d(1, 0, 0);
	std::vector<Util::Vector>::const_iterator point = _shapeA.begin();
	float dotproduct = point->x * d.x + point->y * d.y + point->z * d.z;


	
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
