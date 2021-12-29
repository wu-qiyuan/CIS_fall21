#include <Eigen\Dense>
#include "pch.h"

using namespace Eigen;

Vector3d crossp(Vector3d a, Vector3d b)
{
	return a.cross(b);
}

double dotp(Vector3d a, Vector3d b)
{
	return a.dot(b);
}

Matrix3d invR(Matrix3d R)
{
	return R.transpose();
}

Matrix3d Rot3(Vector3d w, double theta)
{
	Matrix3d R;

	Matrix3d I;
	I << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	Matrix3d hat;
	hat << 0, -w[2], w[1],
		w[2], 0, -w[0],
		-w[1], w[0], 0;

	R = I + sin(theta)*hat + (1 - cos(theta))*hat*hat;

	return R;
}

