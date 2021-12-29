
#include <iostream>
#include <Eigen\Dense>
#include <stdio.h>
#include "vector"
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"
#include "ProjectOnSegment.h"
#include "FindClosestPoint.h"

using namespace Eigen;
using namespace std;

//matching the closest ci using the criteria minimizing ||F_guess*q-ci||
vector<Vector3d> matching(int N_tri, F F_guess, const vector<Vector3d>& d, const vector<Vector3d>& Ver, const vector<VectorXi> &Tri) //q:point set to be registered; c:vertices of surface
{
	int i, j, k;
	int minind,minind_ver; //the index of c that minimizes the distance
	double dis;
	double mindis;
	double dis_ver;
	Vector3d v1, v2, v3, prj;  //3vetex of the triangle
	VectorXi temp6;
	vector<Vector3d> ci;
	vector<Vector3d> matched_c;

	//cout << c.size() << endl;
	for (i = 0; i < d.size(); i++) {  //for each q find closest point
#pragma region simple search
		//Release the memory of ci
		ci.clear();
		//Initialize mindis
		mindis = -5;

		for (j = 0; j < N_tri; j++) {
			MatrixXd coe(2, 1);
			MatrixXd A(3, 2);
			MatrixXd B(3, 1);

			temp6 = Tri[j];
			v1 = Ver[temp6[0]];
			v2 = Ver[temp6[1]];
			v3 = Ver[temp6[2]];
			
			prj = FindClosestPoint(F_guess * d[i], v1, v2, v3);

#pragma region update mindistance and minind
			dis = (F_guess * d[i] - prj).norm();
			if (mindis == -5) {  //initialization of minind and mindis
				mindis = dis;
				minind = j;
			}
			else if (dis <= mindis) {
				mindis = dis;
				minind = j;
			}
#pragma endregion
			ci.push_back(prj);
		}
#pragma endregion

		//matched_index.push_back(minind);
		matched_c.push_back(ci[minind]);

	}
	return matched_c;
}
