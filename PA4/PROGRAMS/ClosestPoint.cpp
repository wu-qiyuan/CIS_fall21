#include <iostream>
#include <Eigen\Dense>
#include <stdio.h>
#include "vector"
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"
#include "ProjectOnSegment.h"

using namespace Eigen;
using namespace std;

Vector3d ClosestPoint(Vector3d d, Vector3d v1, Vector3d v2, Vector3d v3)
{
	Vector3d c, prj, a, b;
	MatrixXd coe(2, 1);
	MatrixXd A(3, 2);
	MatrixXd B(3, 1);

#pragma region calculate lambda and miu
	a = v1 - v3;  //v1 q; v2 r; v3 p
	b = v2 - v3;
	c = d - v3;
	A << a[0], b[0],
		a[1], b[1],
		a[2], b[2];
	//cout << A << endl;
	B << c[0],
		c[1],
		c[2];
	//cout << B << endl;
	//SVD decomposition approach to solve the least square problem A* p = B
	coe = A.bdcSvd(ComputeThinU | ComputeThinV).solve(B);
#pragma endregion

#pragma region find closest point
	if (coe(0, 0) >= 0 && coe(1, 0) >= 0 && (coe(0, 0) + coe(1, 0)) <= 1) {
		prj = v3 + coe(0, 0) * a + coe(1, 0) * b;
	}
	else if (coe(0, 0) < 0 && coe(1, 0) >= 0 && (coe(0, 0) + coe(1, 0)) <= 1) {
		prj = ProjectOnSegment(c, v2, v3);
	}
	else if (coe(0, 0) >= 0 && coe(1, 0) < 0 && (coe(0, 0) + coe(1, 0)) <= 1) {
		prj = ProjectOnSegment(c, v3, v1);
	}
	else if (coe(0, 0) >= 0 && coe(1, 0) >= 0 && (coe(0, 0) + coe(1, 0)) > 1) {
		prj = ProjectOnSegment(c, v1, v2);
	}
	else if (coe(0, 0) < 0 && coe(1, 0) < 0 && (coe(0, 0) + coe(1, 0)) <= 1) {
		prj = v3;  //p
	}
	else if (coe(0, 0) < 0 && coe(1, 0) >= 0 && (coe(0, 0) + coe(1, 0)) > 1) {
		prj = v2;  //r
	}
	else if (coe(0, 0) >= 0 && coe(1, 0) < 0 && (coe(0, 0) + coe(1, 0)) > 1) {
		prj = v1;  //q
	}
#pragma endregion
	return prj;
}