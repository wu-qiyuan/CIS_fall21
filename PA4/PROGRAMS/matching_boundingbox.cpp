
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

//matching the closest ci using the criteria minimizing ||T*q-ci||
vector<Vector3d> matching_boundingbox(int N_tri, F T, const vector<Vector3d>& q, const vector<Vector3d>& Ver, const vector<VectorXi> &Tri) //q:point set to be registered; c:vertices of surface
{
	int i, j, k, n;
	double dis, mindis;
	Vector3d v1, v2, v3, prj;  //3vetex of the triangle
	VectorXi temp6;
	Vector3d ci;
	vector<Vector3d> matched_c;

	//cout << c.size() << endl;
	for (i = 0; i < q.size(); i++) {  //for each q find closest point
		Vector3d d = T * q[i];

		for (j = 0; j < N_tri; j++) {
			MatrixXd coe(2, 1);
			MatrixXd A(3, 2);
			MatrixXd B(3, 1);
			Vector3d L, U;  //lower mindis and upper mindis of box

			temp6 = Tri[j];
			v1 = Ver[temp6[0]];
			v2 = Ver[temp6[1]];
			v3 = Ver[temp6[2]];

			for (n = 0;n < 3;n++) { //find L and U for each triangle
				if (n == 0) {
					L = v1;
					U = v1;
				}else {
					for (k = 0;k < 3;k++) {  //for x, y, z coordinates
						if (Ver[temp6[n]][k] < L[k]) {
							L[k] = Ver[temp6[n]][k];
						}
						if (Ver[temp6[n]][k] > U[k]) {
							U[k] = Ver[temp6[n]][k];
						}
					}
				}
			}
				
			if (j == 0) {  //initialization of mindis
				prj = FindClosestPoint(d, v1, v2, v3);
				dis = (d - prj).norm();
				mindis = dis;
				ci = prj;
			}
			else if (L[0] - mindis <= d[0] && d[0] <= U[0] + mindis && L[1] - mindis <= d[1] && d[1] <= U[1] + mindis && L[2] - mindis <= d[2] && d[2] <= U[2] + mindis) {
				prj = FindClosestPoint(d, v1, v2, v3);
				dis = (d - prj).norm();
				if (dis < mindis) {
					mindis = dis;
					ci = prj;
				}
			}
		}
#pragma endregion

		matched_c.push_back(ci);

	}
	return matched_c;
}
