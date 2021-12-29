#include "pch.h"
#include "CatesianMathBasic.h"
#include "vector"
#include "stdio.h"
#include <stdlib.h>
#include <Eigen\Dense>

using namespace std;
MatrixXd ScaleToBox(int N, int N_frames, int N_x, vector<Vector3d>& q);
