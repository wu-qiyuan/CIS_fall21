#include "pch.h"
#include "CatesianMathBasic.h"
#include "vector"
#include "stdio.h"
#include <stdlib.h>
#include <iostream>
#include <Eigen\Dense>

using namespace std;

/*****Determine a bounding box to scale q values*****/
MatrixXd ScaleToBox(int N, int N_frames, int N_x, vector<Vector3d> &q) {
	MatrixXd qs_matrix;  //Copy the data of q vector into a matrix(N_frames * N_C, 3)
	qs_matrix = Eigen::Matrix<double, Dynamic, 3>();
	qs_matrix.resize(N_frames * N_x, 3);
	for (int i = 0; i < N_frames; i++) {
		vector<Vector3d>::const_iterator First = q.begin() + floor(i * N_x);
		vector<Vector3d>::const_iterator Second = q.begin() + floor(i * N_x) + N_x;
		vector<Vector3d> qi(First, Second);
		for (int j = 0; j < N_x; j++) {
			Vector3d temp = qi[j];
			int n = i * N_x + j;     //row number of q_mtrix
			qs_matrix(n, 0) = temp[0];   //Put x value into q_matrix_x
			qs_matrix(n, 1) = temp[1];   //Put y value into q_matrix_y
			qs_matrix(n, 2) = temp[2];   //Put z value into q_matrix_z
		}
	}
	Vector3d min = qs_matrix.colwise().minCoeff();  //Compute min value of each column in qs_matrix
	Vector3d max = qs_matrix.colwise().maxCoeff();  //Compute max value of each column in qs_matrix
	//cout << "min = " << min << endl;
	//cout << "max = " << max << endl;
	MatrixXd us_matrix;  //Copy the data of us into a matrix(N_frames * N_C, 3)
	us_matrix = Eigen::Matrix<double, Dynamic, 3>();
	us_matrix.resize(N_frames * N_x, 3);
	for (int i = 0; i < N_frames * N_x; i++) {
		/*us_matrix(i, 0) = (qs_matrix(i, 0) - min(0)) / (max(0) - min(0));
		us_matrix(i, 1) = (qs_matrix(i, 1) - min(1)) / (max(1) - min(1));
		us_matrix(i, 2) = (qs_matrix(i, 2) - min(2)) / (max(2) - min(2));*/
		us_matrix(i, 0) = (qs_matrix(i, 0) - 80) / (700 - 80);
		us_matrix(i, 1) = (qs_matrix(i, 1) - 80) / (700 - 80);
		us_matrix(i, 2) = (qs_matrix(i, 2) - 80) / (700 - 80);
	}
	return us_matrix;
}