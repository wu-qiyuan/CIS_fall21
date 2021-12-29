#include <iostream>
#include <Eigen\Dense>
#include <stdio.h>
#include "vector"
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"

using namespace Eigen;
using namespace std;

Vector3d ProjectOnSegment(const Vector3d& c, const Vector3d& v1, const Vector3d& v2)  //project c on the segment of v1 and v2
{
	double lambda = dotp((c - v1), (v2 - v1)) / dotp((v2 - v1), (v2 - v1));
	Vector3d prj = v1 + lambda * (v2 - v1);
	return prj;
}