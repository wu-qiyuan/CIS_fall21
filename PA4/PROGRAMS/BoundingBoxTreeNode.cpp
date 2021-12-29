#include <Eigen\Dense>
#include "pch.h"
#include "vector"
#include "BoundingBoxTreeNode.h"
#include "ProjectOnSegment.h"
#include "ClosestPoint.h"

using namespace Eigen;
using namespace std;


BoundingBox::BoundingBox(Vector3d v1, Vector3d v2, Vector3d v3) 
{
	V1 = v1; V2 = v2; V3 = v3;
	vector<Vector3d> v;
	v.push_back(v1); v.push_back(v2); v.push_back(v3);

	//Compute Upper bound and Lower bound of the bounding box
	for (int i = 0; i < 3; i++) {
		if (i == 0) { UB = v[i]; LB = v[i]; }
		else
		{
			for (int j = 0; j < 3; j++) {
				if (v[i][j] < LB[j]) { LB[j] = v[i][j]; }
				if (v[i][j] > UB[j]) { UB[j] = v[i][j]; }
			}
		}
	}
	Center = LB + (UB - LB) / 2.0;
}



BoundingBoxTreeNode::BoundingBoxTreeNode(BoundingBox** BB, int nB)
{
	Boxes = BB; nBoxes = nB;
	for (int i = 0; i < nBoxes; i++) {
		BoundingBox box = *Boxes[i];
		//cout << i << endl;
		//cout << box.UB.transpose() << endl;
		//cout << box.LB.transpose() << endl;
		if (i == 0) { UB = box.UB; LB = box.LB; }
		else
		{
			for (int j = 0; j < 3; j++) {
				if (box.LB[j] < LB[j]) { LB[j] = box.LB[j]; }
				if (box.UB[j] > UB[j]) { UB[j] = box.UB[j]; }
			}
		}
	}
	Center = LB + (UB - LB) / 2.0;   // This will be the splitting point
}

void BoundingBoxTreeNode::ConstructSubtrees()
{
	int nnn = 0, npn = 0, npp = 0, nnp = 0, pnn = 0, ppn = 0, ppp = 0, pnp = 0; // number of boxes in each subtree
	SplitSort(Center, Boxes, nnn, npn, npp, nnp, pnn, ppn, ppp, pnp);
	SubTrees[0][0][0] = new BoundingBoxTreeNode(&Boxes[0], nnn);
	SubTrees[0][1][0] = new BoundingBoxTreeNode(&Boxes[nnn], npn);
	SubTrees[0][1][1] = new BoundingBoxTreeNode(&Boxes[nnn + npn], npp);
	SubTrees[0][0][1] = new BoundingBoxTreeNode(&Boxes[nnn + npn + npp], nnp);
	SubTrees[1][0][0] = new BoundingBoxTreeNode(&Boxes[nnn + npn + npp + nnp], pnn);
	SubTrees[1][1][0] = new BoundingBoxTreeNode(&Boxes[nnn + npn + npp + nnp + pnn], ppn);
	SubTrees[1][1][1] = new BoundingBoxTreeNode(&Boxes[nnn + npn + npp + nnp + pnn + ppn], ppp);
	SubTrees[1][0][1] = new BoundingBoxTreeNode(&Boxes[nnn + npn + npp + nnp + pnn + ppn + ppp], pnp);
}

void BoundingBoxTreeNode::SplitSort(Vector3d SplittingPoint, BoundingBox** Boxes, int& nnn, int& npn, int& npp, int& nnp, int& pnn, int& ppn, int& ppp, int& pnp)
{ 
	vector<BoundingBox> bucket1, bucket2, bucket3, bucket4, bucket5, bucket6, bucket7, bucket8;
	vector<BoundingBox*> bucket;
	vector<int> num;
	for (int i = 0; i < nBoxes; i++) {
		BoundingBox box = *Boxes[i];
		if (box.Center[0] > Center[0] && box.Center[1] > Center[1] && box.Center[2] > Center[2]) { nnn++; bucket1.push_back(box); }
		if (box.Center[0] > Center[0] && box.Center[1] < Center[1] && box.Center[2] > Center[2]) { npn++; bucket2.push_back(box); }
		if (box.Center[0] > Center[0] && box.Center[1] < Center[1] && box.Center[2] < Center[2]) { npp++; bucket3.push_back(box); }
		if (box.Center[0] > Center[0] && box.Center[1] > Center[1] && box.Center[2] < Center[2]) { nnp++; bucket4.push_back(box); }
		if (box.Center[0] < Center[0] && box.Center[1] > Center[1] && box.Center[2] > Center[2]) { pnn++; bucket5.push_back(box); }
		if (box.Center[0] < Center[0] && box.Center[1] < Center[1] && box.Center[2] > Center[2]) { ppn++; bucket6.push_back(box); }
		if (box.Center[0] < Center[0] && box.Center[1] < Center[1] && box.Center[2] < Center[2]) { ppp++; bucket7.push_back(box); }
		if (box.Center[0] < Center[0] && box.Center[1] > Center[1] && box.Center[2] < Center[2]) { pnp++; bucket8.push_back(box); }
	}
	//Push the number of boundingbox of each subtree into a vector
	num.push_back(nnn); num.push_back(npn); num.push_back(npp); num.push_back(nnp); 
	num.push_back(pnn); num.push_back(ppn); num.push_back(ppp); num.push_back(pnp);
	//Connect all of the buckets into one bucket
	bucket1.insert(bucket1.end(), bucket2.begin(), bucket2.end());
	bucket1.insert(bucket1.end(), bucket3.begin(), bucket3.end());
	bucket1.insert(bucket1.end(), bucket4.begin(), bucket4.end());
	bucket1.insert(bucket1.end(), bucket5.begin(), bucket5.end());
	bucket1.insert(bucket1.end(), bucket6.begin(), bucket6.end());
	bucket1.insert(bucket1.end(), bucket7.begin(), bucket7.end());
	bucket1.insert(bucket1.end(), bucket8.begin(), bucket8.end());
	int count = 0;
	for (int j = 0; j < 8; j++) { 
		if (j > 0) { count += num[j - 1]; }
		for (int k = 0; k < num[j]; k++) {
			*Boxes[count + k] = bucket1[count + k];
		}
	}
}



void BoundingBoxTreeNode::FindClosestPoint(Vector3d v, double& bound, Vector3d& closest)
{
	double dist = bound;
	Vector3d cp;   //Computed closest point
	if (v[0] > (UB[0] + bound) || v[0] < (LB[0] - bound)) return; 
	if (v[1] > (UB[1] + bound) || v[1] < (LB[1] - bound)) return; 
	if (v[2] > (UB[2] + bound) || v[2] < (LB[2] - bound)) return;
	int minCount = 50;
	int minDiag = 0.001;
	if (nBoxes <= minCount || (UB - LB).norm() <= minDiag)
	{
		HaveSubtrees = 0;
		for (int i = 0; i < nBoxes; i++) {
			BoundingBox Box = *Boxes[i];
			Vector3d cp = ClosestPoint(v, Box.V1, Box.V2, Box.V3);
			dist = (cp - v).norm();
			if (dist < bound)
			{
				bound = dist; closest = cp;
			}
		}
	}
	else {
		ConstructSubtrees();
		SubTrees[0][0][0]->FindClosestPoint(v, bound, closest);
		SubTrees[0][1][0]->FindClosestPoint(v, bound, closest);
		SubTrees[0][1][1]->FindClosestPoint(v, bound, closest);
		SubTrees[0][0][1]->FindClosestPoint(v, bound, closest);
		SubTrees[1][0][0]->FindClosestPoint(v, bound, closest);
		SubTrees[1][1][0]->FindClosestPoint(v, bound, closest);
		SubTrees[1][1][1]->FindClosestPoint(v, bound, closest);
		SubTrees[1][0][1]->FindClosestPoint(v, bound, closest);
	}
}

BoundingBoxTreeNode::~BoundingBoxTreeNode()
{
	int i, j, k;
	for (i = 0; i < 2; i++) {
		for (j = 0; j < 2; j++) {
			for (k = 0; k < 2; k++) {
				delete SubTrees[i][j][k];
			}
		}
	}
	//delete
}





