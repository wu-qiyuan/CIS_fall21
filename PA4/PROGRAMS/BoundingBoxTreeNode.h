#include <Eigen\Dense>
#include "pch.h"
#include "vector"

using namespace Eigen;
using namespace std;

class BoundingBox {
public:
	Vector3d UB, LB, V1, V2, V3;
	Vector3d Center;
	BoundingBox(Vector3d v1, Vector3d v2, Vector3d v3);
};


class BoundingBoxTreeNode {
public:
	Vector3d Center;   //Splitting point
	Vector3d UB;       //Corners of box
	Vector3d LB;
	int HaveSubtrees;
	int nBoxes;
	BoundingBoxTreeNode* SubTrees[2][2][2];
	BoundingBox** Boxes;

	BoundingBoxTreeNode(BoundingBox** BB, int nB);
	void ConstructSubtrees();
	void SplitSort(Vector3d SplittingPoint, BoundingBox** Boxes, int& nnn, int& npn, int& npp, int& nnp, int& pnn, int& ppn, int& ppp, int& pnp);
	void FindClosestPoint(Vector3d v, double& bound, Vector3d& closest);
	~BoundingBoxTreeNode();
}; 
