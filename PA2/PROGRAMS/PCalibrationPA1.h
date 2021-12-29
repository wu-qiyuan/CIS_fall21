
#include <Eigen\Dense>
#include "vector"
#include "pch.h"
#include "F.h"
#include <iostream>
#include "vector"
#include "pivot.h"


VectorXd PCalibrationPA1(const std::vector<Vector3d> &G, const int N_G, const int N_frames, std::vector<Vector3d> &g);  //read data set of G

