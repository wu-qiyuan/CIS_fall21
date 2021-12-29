#include "pch.h"
#include "CatesianMathBasic.h"
#include "vector"
#include "stdio.h"
#include <stdlib.h>
#include <Eigen\Dense>
#include "ScaleToBox.h"
#include <iostream>
#include <fstream>


using namespace std;


int binomialCoefficients(int n, int k) {
	if (k == 0 || k == n)
		return 1;
	return binomialCoefficients(n - 1, k - 1) + binomialCoefficients(n - 1, k);
}

MatrixXd Fijk(int N, double ux, double uy, double uz) {
	//MatrixXd bi_coefficient(1, 6);
	//bi_coefficient << 1, 5, 10, 10, 5, 1;
	MatrixXd F_us;
	double F_us_temp;
	F_us = Eigen::Matrix<double, Dynamic, Dynamic>();
	F_us.resize(1, (N + 1) * (N + 1) * (N + 1));
	for (int i = 0; i < N + 1; i++) {
		for (int j = 0; j < N + 1; j++) {
			for (int k = 0; k < N + 1; k++) {
				int bi_coefficient_i = binomialCoefficients(N, i);
				int bi_coefficient_j = binomialCoefficients(N, j);
				int bi_coefficient_k = binomialCoefficients(N, k);
				double Bi = (double)bi_coefficient_i * pow(1 - ux, N - i) * pow(ux, i);
				double Bj = (double)bi_coefficient_j * pow(1 - uy, N - j) * pow(uy, j);
				double Bk = (double)bi_coefficient_k * pow(1 - uz, N - k) * pow(uz, k);
				F_us_temp = Bi * Bj * Bk;
				int square_coe = (int)pow(N + 1, 2);
				int linear_coe = N + 1;
				int n = square_coe * i + linear_coe * j + k;
				F_us(0, n) = F_us_temp;
			}
		}
	}
	return F_us;
}

/*Distortion correction function*/
vector<Vector3d> CorrectDistortion(int N, int N_frames, int N_x, vector<Vector3d>& q, MatrixXd c_matrix)
{
	vector<Vector3d> p;
	/*****Determine a bounding box to scale q values*****/
	MatrixXd us_matrix = ScaleToBox(N, N_frames, N_x, q);
	/*Construct a "tensor form" interpolation polynomial */
	MatrixXd F_us;
	F_us = Eigen::Matrix<double, Dynamic, 216>();
	F_us.resize(N_frames * N_x, 216);
	for (int i = 0; i < N_frames * N_x; i++) {
		MatrixXd temp = Fijk(N, us_matrix(i, 0), us_matrix(i, 1), us_matrix(i, 2));    //Compute Fijk for each us
		F_us.block<1, 216>(i, 0) = temp;
	}
	/*Compute corrected p*/
	MatrixXd p_matrix = F_us * c_matrix;
	for (int i = 0; i < N_frames * N_x; i++) {
		Vector3d temp;
		temp[0] = p_matrix(i, 0);
		temp[1] = p_matrix(i, 1);
		temp[2] = p_matrix(i, 2);
		p.push_back(temp);
	}
	return p;
}

