#include <Eigen\Dense>
#include "pch.h"

using namespace Eigen;

Vector3d crossp(Vector3d a, Vector3d b);

double dotp(Vector3d a, Vector3d b);

Matrix3d invR(Matrix3d R);

Matrix3d Rot3(Vector3d w, double theta);
