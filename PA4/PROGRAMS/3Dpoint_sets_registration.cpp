
#include <iostream>
#include <Eigen\Dense>
#include <stdio.h>
#include "vector"
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"

using namespace Eigen;
using namespace std;

/*To solve the 3D point set to 3D point set registration problem, we have to solve the equation: R¡¤ a + p = b, where a and b is the known point sets.*/
/*Input: 3D point set a, 3D point set b*/
/*Output: Frame(R, p)*/

F Point3D_Sets_Registration(vector<Vector3d> &a, vector<Vector3d> &b)
{
	/*****Step 1: Compute*****/

	  /*Compute the average value of 3D point set a and b*/
	Vector3d a_sum(0, 0, 0);                       //summed value of points in a
	Vector3d b_sum(0, 0, 0);                      //summed value of points in b
	Vector3d a_ave;                                    //average value of points in a
	Vector3d b_ave;                                   //average value of points in b
	int a_size = a.size();                             //The number of points in point set a
	int b_size = b.size();                            //The number of points in point set b
	for (int i = 0; i < a_size; i++) {
		Vector3d temp = a[i];
		//		cout << a[i] << endl << endl;
		for (int j = 0; j < 3; j++) {
			a_sum[j] += temp[j];
		}
	}
	a_ave = a_sum / a_size;
	//cout << a_ave << endl;
	for (int i = 0; i < b_size; i++) {
		//		cout << b[i] << endl << endl;
		Vector3d temp = b[i];
		for (int j = 0; j < 3; j++) {
			b_sum[j] += temp[j];
		}
	}
	b_ave = b_sum / b_size;
	//cout << b_ave << endl;

	/*Compute new value of points in a and b*/
	vector<Vector3d> a_new = a;
	vector<Vector3d> b_new = b;
	for (int i = 0; i < a_size; i++) {
		a_new[i] = a[i] - a_ave;
		//		cout << a_new[i] << endl << endl;
	}
	for (int i = 0; i < b_size; i++) {
		b_new[i] = b[i] - b_ave;
		//		cout << b_new[i] << endl << endl;
	}

	/*****Step 2: Find R that minimizes*****/
	/*Direct techniques to solve for R (Quaternion method)*/
	/*Compute H*/
	Matrix3d H;                  //Initialize the zero matrix H
	H << 0, 0, 0,
		0, 0, 0,
		0, 0, 0;
	Matrix3d h;                   //Matrix used in each step of sum
	for (int i = 0; i < a_size; i++) {
		Vector3d h_a = a_new[i];
		Vector3d h_b = b_new[i];
		h(0, 0) = h_a[0] * h_b[0];
		h(0, 1) = h_a[0] * h_b[1];
		h(0, 2) = h_a[0] * h_b[2];
		h(1, 0) = h_a[1] * h_b[0];
		h(1, 1) = h_a[1] * h_b[1];
		h(1, 2) = h_a[1] * h_b[2];
		h(2, 0) = h_a[2] * h_b[0];
		h(2, 1) = h_a[2] * h_b[1];
		h(2, 2) = h_a[2] * h_b[2];
		//		cout << h << endl << endl;
		H += h;
	}
	//Compute G
	MatrixXd G(4, 4);
	G(0, 0) = H.trace();
	MatrixXd delta(3, 1);
	delta << H(1, 2) - H(2, 1), H(2, 0) - H(0, 2), H(0, 1) - H(1, 0);
	//cout << delta << endl;
	Matrix3d G_1_1 = H + H.transpose() - H.trace() * MatrixXd::Identity(3, 3);
	G.block<3, 3>(1, 1) = G_1_1;
	G.block<1, 3>(0, 1) = delta.transpose();
	G.block<3, 1>(1, 0) = delta;
	//cout << "G = " << endl;
	//cout << G << endl;
	//Compute eigen value decomposition of G: (A=VDV?1 is called the eigendecomposition).Here A = G;
	EigenSolver<MatrixXd> ces;
	ces.compute(G);
	MatrixXcd lamada_cd = ces.eigenvalues();         //Compute the eigen values
	//cout << lamada_cd << endl;
	MatrixXd lamada(4, 1);
	for (int i = 0; i < 4; i++) {
		lamada(i, 0) = lamada_cd(i, 0).real();
	}
	//cout << "lamada = " << endl;
	//cout << lamada << endl;
	MatrixXcd Q_cd = ces.eigenvectors();             //Compute the eigen vectors
	//cout << Q_cd << endl;
	MatrixXd Q(4, 4);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			Q(i, j) = Q_cd(i, j).real();
		}
	}
	//cout << "Q = " << endl;
	//cout << Q << endl;
	MatrixXd::Index maxRow, maxCol;
	double max = lamada.maxCoeff(&maxRow, &maxCol);  //Find the position of max lamada
	MatrixXd q = Q.col(maxRow);                      //The eigen vector corresponding to the max eigen value
	//cout << "q = " << endl << q << endl;
	Matrix3d R;
	R(0, 0) = pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2);
	R(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
	R(0, 2) = 2 * (q(1) * q(3) + q(0) * q(2));
	R(1, 0) = 2 * (q(1) * q(2) + q(0) * q(3));
	R(1, 1) = pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) - pow(q(3), 2);
	R(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1));
	R(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
	R(2, 1) = 2 * (q(2) * q(3) + q(0) * q(1));
	R(2, 2) = pow(q(0), 2) - pow(q(1), 2) - pow(q(2), 2) + pow(q(3), 2);
	//cout << "R = " << endl;
	//cout << R << endl << endl;

	/*****Step 3: Find p*****/
	Vector3d p = b_ave - R * a_ave;
	//cout << "p = " << endl;
	//cout << p << endl;
	//cout << "||q|| = " << pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2) << endl;

	/*****Step 4: Desired transformation*****/
	F F_registration(R, p);
	return F_registration;
}
