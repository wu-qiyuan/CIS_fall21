#include "pch.h"
#include "CatesianMathBasic.h"
#include "vector"
#include "stdio.h"
#include <stdlib.h>
#include <Eigen\Dense>
#include "ScaleToBox.h"

int binomialCoefficients(int n, int k);
MatrixXd Fijk(int N, double ux, double uy, double uz);
vector<Vector3d> CorrectDistortion(int N, int N_frames, int N_x, vector<Vector3d>& q, MatrixXd c_matrix);