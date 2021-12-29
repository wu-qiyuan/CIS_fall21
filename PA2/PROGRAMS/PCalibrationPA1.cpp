
#include <Eigen\Dense>
#include "vector"
#include "pch.h"
#include "F.h"
#include <iostream>
#include "vector"
#include "pivot.h"
#include "3Dpoint_sets_registration.h"

using namespace std;

VectorXd PCalibrationPA1(const std::vector<Vector3d> &G, const int N_G, const int N_frames, std::vector<Vector3d> &g)  //read data set of G
{
	//value of g would change in this function
	//Q5.a
	Vector3d sum_G(0, 0, 0);
	Vector3d G_0;  //average position of frame 0

	int i, j;

	for (i = 0; i < 1; i++) {  //use data of frame 0
		vector<Vector3d>::const_iterator First = G.begin() + floor(i * N_G);
		vector<Vector3d>::const_iterator Second = G.begin() + floor(i * N_G) + N_G;
		vector<Vector3d> Gi(First, Second);
		for (j = 0;j < N_G;j++) {
			sum_G += Gi[j];
		}
		G_0 = (1.0 / N_G) * sum_G;

		for (j = 0;j < N_G;j++) {
			g.push_back(G[j] - G_0);
		}
	}

	//Q5.b  F_G[k=1 to N_frame] is the solution
	vector<F> F_G;
	for (i = 1; i < N_frames; i++) {  //use data of frame 1 to last frame
		vector<Vector3d>::const_iterator First = G.begin() + floor(i * N_G);
		vector<Vector3d>::const_iterator Second = G.begin() + floor(i * N_G) + N_G;
		vector<Vector3d> Gi(First, Second);

		F_G.push_back(Point3D_Sets_Registration(g, Gi));  //F_regist*g=Gi
	}

	//Q5.c
	VectorXd p_G(6);
	p_G << pivot(F_G);
	return p_G;  //return p as [pt;ppivot]
}

