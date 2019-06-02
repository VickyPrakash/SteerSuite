/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"
#include <math.h>

SteerLib::GJK_EPA::GJK_EPA()
{
}


Util::Vector GetFarthestPoint(const std::vector<Util::Vector>& _shape, Util::Vector& d)		//Find the farthest point in a polygon in the direction of d
{
	float dp_max = d * _shape[0];
	int index = 0;

	for (int i = 1; i < _shape.size(); i++)
	{
		float dp = d * _shape[i];
		if (dp_max < dp)
		{
			dp_max = dp;
			index = i;
		}
	}
	return _shape[index];
}


Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector& d)	//Find the support vector
{
	Util::Vector p1 = GetFarthestPoint(_shapeA, d);		//Farthest Point in _shapeA in the direction of d
	Util::Vector dn = -1 * d;
	Util::Vector p2 = GetFarthestPoint(_shapeB, dn);	//Farthest Point in _shapeB in the negative direction of d
	Util::Vector p3 = p1 - p2; //Minkowski Difference
	return p3;
}


bool ContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& d)
{
	Util::Vector a = simplex.back();
	Util::Vector an = -1 * a;

	if (simplex.size() == 3)
	{
		//Triangle
		Util::Vector b = simplex[1];
		Util::Vector c = simplex[0];

		Util::Vector ab = b - a;
		Util::Vector ac = c - a;
		//Implementing Triple product to find the normals to ab and ac
		float dot_ab = ac.x * ab.z - ab.x * ac.z;
		float dot_ac = ab.x* ac.z - ab.z* ac.x;
		Util::Vector ab_normal = Util::Vector(-ab.z * dot_ab, ab.y, dot_ab * ab.x);
		Util::Vector ac_normal = Util::Vector(-ac.z * dot_ac, ac.y, dot_ac * ac.x);
		
		if (ab_normal * an > 0) {
			simplex.erase(simplex.begin() + 0);
			d = ab_normal;
		}
		else {
			if (ac_normal * an > 0) {
				simplex.erase(simplex.begin() + 1);
				d = ac_normal;
			}
			else {
				return true;
			}
		}
	}
	else {
		//line segment
		Util::Vector b = simplex[0];
		Util::Vector ab = b - a;
		float dot = ab.x * an.z - an.x * ab.z;
		Util::Vector ab_normal = Util::Vector(-dot*ab.z, ab.y, dot*ab.x);
		d = ab_normal;
		if (ab_normal * an < 0) {
			d = -1 * ab_normal;
		}
	}
	return false;

}


bool GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex)
{
	//Trying to implement GJK on concave polygons
	//std::vector<Util::Vector> tempA = _shapeA;
	//int index = 1;
	//float temp = tempA[1].x;
	//Util::Vector d = Util::Vector(0, 0, 0);
	//Util::Vector d = Util::Vector(-1, 0, 1);
	Util::Vector d = Util::Vector(1, 0, 1);	//direction vector
	/*for (int i = 1; i < tempA.size() - 1; i++)
	{
		if (temp > tempA[i].x)
		{
			temp = tempA[i].x;
			index = i;
		}
	}
	*/
	simplex.push_back(support(_shapeA, _shapeB, d));
	Util::Vector dn = -1 * d;
	while (true)
	{
		simplex.push_back(support(_shapeA, _shapeB, dn));
		if (simplex.back() * dn <= 0)
		{
			return false;
		}
		else {
			if (ContainsOrigin(simplex, dn))
			{
				return true;
			}
		}
	}
}

void FindClosestEdge(std::vector<Util::Vector>& simplex, float& distance, Util::Vector& normal, int& index)
{
	distance = 100000.0;
	for (int i = 0; i < simplex.size(); i++)
	{
		int j;
		if (i + 1 == simplex.size()) {
			j = 0;
		}
		else {
			j = i + 1;
		}

		Util::Vector a = simplex[i];
		Util::Vector b = simplex[j];
		Util::Vector edge = b - a;
		Util::Vector n = a * (edge*edge) - edge * (edge*a);
		float temp = (n.x * n.x) + (n.y * n.y) + (n.z * n.z);
		Util::Vector normalized_n = n / sqrt(temp);
		float dist = normalized_n * a;
		if (dist < distance)
		{
			distance = dist;
			normal = normalized_n;
			index = j;
		}


	}
}


bool EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex)
{
	float distance;
	Util::Vector normal;
	int index;
	while (true) {
		FindClosestEdge(simplex, distance, normal, index);
		Util::Vector p = support(_shapeA, _shapeB, normal);
		float dist1 = p * normal;
		if (dist1 - distance <= 0) {
			return_penetration_vector = normal;
			return_penetration_depth = dist1;
			return true;
		}
		else {
			simplex.insert(simplex.begin() + index, p);
		}
	}

}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool IsCollision = GJK(_shapeA, _shapeB, simplex);

	if (IsCollision)
	{
		//std::cout << "entering";
		EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
	}
	return IsCollision;
}

/*bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool IsCollision = false;
	std::vector<Util::Vector> triaA;
	std::vector<Util::Vector> triaB;

	for (int i = 0; i < _shapeA.size() - 2; i++)
	{
		for (int j = 0; j < _shapeB.size() - 2; j++)
		{
			triaA.push_back(_shapeA[i]);
			triaA.push_back(_shapeA[i + 1]);
			triaA.push_back(_shapeA[i + 2]);
			triaB.push_back(_shapeB[j]);
			triaB.push_back(_shapeB[j + 1]);
			triaB.push_back(_shapeB[j + 2]);

			bool temp = GJK(triaA, triaB, simplex);
			if (temp)
			{
				IsCollision = temp;
			}
			triaA.clear();
			triaB.clear();
		}

	}
	//temp = true;


	if (IsCollision)
	{
		//std::cout << "entering";
		EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
	}
	return IsCollision;
} */
