#include <iostream>
#include <Eigen\Dense>
#include <stdio.h>
#include "vector"
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"

using namespace Eigen;
using namespace std;

vector<Vector3d> matching_boundingbox(int N_tri, F F_guess, const vector<Vector3d>& d, const vector<Vector3d>& Ver, const vector<VectorXi>& Tri);
