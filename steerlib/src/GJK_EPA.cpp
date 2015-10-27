/*!
*
* \author VaHiD AzIzI
*
*/

#include <limits>
#include "obstacles/GJK_EPA.h"

typedef struct Edge {
	float distance;
	Util::Vector normal;
	unsigned int index;
}Edge;

SteerLib::GJK_EPA::GJK_EPA()
{
}

float DotProduct(Util::Vector A, Util::Vector B)
{
	return (A.x * B.x) + (A.y * B.y) + (A.z * B.z);
}

Util::Vector TripleProduct(Util::Vector A, Util::Vector B, Util::Vector C) {
	// Formula: (A  x  B) x C =   B(C  dot  A) -  A(C  dot  B)
	// Example: (AB x A0) x AB = A0(AB dot AB) - AB(AB dot A0)

	float CdotA = DotProduct(C, A);
	float CdotB = DotProduct(C, B);

	return B*CdotA - A*CdotB;
}

Util::Vector GetFarthestPoint(const std::vector<Util::Vector>& _shape, Util::Vector d)
{
	// Didn't produce the right resul if maxdot = 0;
	float maxdot = -100000000000;
	float dotproduct;
	Util::Vector farthest;

	for (int i = 0; i < _shape.size(); ++i) {
		dotproduct = DotProduct(_shape[i], d);

		if (dotproduct > maxdot) {
			maxdot = dotproduct;
			farthest = _shape[i];
		}
	}
	
	return farthest;
}

Util::Vector Support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector d)
{
	return GetFarthestPoint(_shapeA, d) - GetFarthestPoint(_shapeB, -d);
}

bool SimplexOrigins(std::vector<Util::Vector>& _simplex, Util::Vector& d)
{
	// Set A to last thing added to Simplex
	Util::Vector A = _simplex[_simplex.size() - 1];
	Util::Vector A0 = -A;

	if (_simplex.size() == 3) {
		// Triangle
		
		// Get points B and C
		Util::Vector B = _simplex.at(0);
		Util::Vector C = _simplex.at(1);

		// Get Edges
		Util::Vector AB = B - A;
		Util::Vector AC = C - A;

		// Get Normals
		Util::Vector ABPerp = TripleProduct(AC, AB, AB);
		Util::Vector ACPerp = TripleProduct(AB, AC, AC);

		if (DotProduct(ABPerp, A0) > 0) {
			// Remove C
			_simplex.erase(_simplex.begin() + 1);
			
			// Reset direction to ABPerp
			d = ABPerp;
		}
		else {
			if (DotProduct(ACPerp, A0) > 0) {
				// Remove B
				_simplex.erase(_simplex.begin());

				// Reset direction to ACPerp
				d = ACPerp;
			}
			else {
				// Origin is here
				return true;
			}
		}
	}
	else {
		// Line 

		// Get point B
		Util::Vector B = _simplex.at(0);

		// Get Edge
		Util::Vector AB = B - A;
	
		// Get Normal
		Util::Vector ABPerp = TripleProduct(AB, A0, AB);

		// Reset direction to ABPerp
		d = ABPerp;
	}

	return false;
}

bool GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> _simplex;

	// Get random direction
	Util::Vector d(1, 0, 0);

	// Get first point for minkowski difference
	Util::Vector A = Support(_shapeA, _shapeB, d);
	_simplex.push_back(A);

	// Negate d for next point
	d = -d;

	while (true) {
		// Get next point for simplex
		Util::Vector B = Support(_shapeA, _shapeB, d);
		_simplex.push_back(B);

		// Make sure B passes origin via dot product
		float dotproduct = DotProduct(B, d);
		if (dotproduct <= 0) {
			// Didn't pass origin so it probably won't ever
			return false;
		}
		else {
			// You made it this far, so check the origin
			if (SimplexOrigins(_simplex, d)) {
				// Collides!
				return true;
			}
		}
	}
	return false;
}

Edge findClosestEdge(std::vector<Util::Vector> polygon) {
	// Returns the edge of the polygon which is closest to the origin.

	Edge closest;
	closest.distance = std::numeric_limits<float>::max();

	for (unsigned int i = 0; i < polygon.size(); ++i) {
		unsigned int j = (i + 1 == polygon.size()) ? 0 : i + 1;

		Util::Vector a = polygon[i];
		Util::Vector b = polygon[j];
		Util::Vector e = b - a;
		Util::Vector oa = a; // Think of this as a - origin
		Util::Vector n = TripleProduct(e, oa, e);
		n = Util::normalize(n);

		float d = DotProduct(n, a);

		if (d < closest.distance) {
			closest.distance = d;
			closest.normal = n;
			closest.index = j;
		}
	}

	return closest;
}

void SteerLib::GJK_EPA::EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{

	std::vector<Util::Vector> simplex = _simplex; // Copy the original so we can expand it.
	Util::Vector normal;
	Edge closestEdge;

	float epsilon = 0.0001; // Should be a small number.

	while (true)
	{
		closestEdge = findClosestEdge(simplex);
		Util::Vector supportVector = Support(_shapeA, _shapeB, closestEdge.normal);

		float d = DotProduct(supportVector, closestEdge.normal);

		if (d - closestEdge.distance < epsilon)
		{
			return_penetration_vector = closestEdge.normal;
			return_penetration_depth = d;
			return;
		}

		else
			simplex.insert(simplex.begin() + closestEdge.index, supportVector);
	}

}

//Look at the GJK_EPA.h header file for documentation and instructions
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
