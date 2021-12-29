#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"
#include "pivot.h"
#include "3Dpoint_sets_registration.h"
#include "vector"
#include "stdio.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <Eigen\Dense>
#include "ScaleToBox.h"
#include "CorrectDistortion.h"
#include "PCalibrationPA1.h"

int main()
{
#pragma region Define problem set variables
	//Define problem set variables
	int N_D, N_A, N_C, N_frames_cal, N_G, N_H, N_D_2, N_frames_empivot, N_B, N_frames_nav;
	vector<Vector3d> d;
	vector<Vector3d> a;
	vector<Vector3d> c;
	vector<Vector3d> D;
	vector<Vector3d> A;
	vector<Vector3d> C;
	vector<Vector3d> G;
	vector<Vector3d> H;
	vector<Vector3d> D_2;
	vector<Vector3d> G_fidu;
	vector<Vector3d> b;
	vector<Vector3d> G_nav;
#pragma endregion

#pragma region Define global assistant variables
	int i, j, k;
	double x, y, z;
	char s[50];
	string cas;
	Vector3d temp;
	char read[50];
#pragma endregion

	cout << "Please type in case num. (e.g. pa2-debug-a, etc..):";
	cin >> cas;
	//cas = "pa1-debug-a";
	cout << "Processing: " << cas << endl;

#pragma region read files
#pragma region read calbody file
	//Read calbody file
	FILE* calbody;
	strcpy(read, ("..\\INPUT\\" + cas + "-calbody.txt").c_str());
	calbody = fopen(read, "r");
	fscanf(calbody, "%d, %d, %d, %s", &N_D, &N_A, &N_C, s);
	cout << "reading:" << s << endl;
	//read d_i
	for (i = 0; i < N_D; i++)
	{
		fscanf(calbody, "%lg, %lg, %lg", &x, &y, &z);
		temp << x, y, z;
		d.push_back(temp);
	}
	//read a_i
	for (i = 0; i < N_A; i++)
	{
		fscanf(calbody, "%lg, %lg, %lg", &x, &y, &z);
		temp << x, y, z;
		a.push_back(temp);
	}
	//read c_i
	for (i = 0; i < N_C; i++)
	{
		fscanf(calbody, "%lg, %lg, %lg", &x, &y, &z);
		temp << x, y, z;
		c.push_back(temp);
	}
	fclose(calbody);
#pragma endregion

#pragma region read calreadings file
	//Read calreadings file
	FILE* calreadings;
	strcpy(read, ("..\\INPUT\\" + cas + "-calreadings.txt").c_str());
	calreadings = fopen(read, "r");
	fscanf(calreadings, "%d, %d, %d, %d, %s", &N_D, &N_A, &N_C, &N_frames_cal, s);
	cout << "reading:" << s << endl;
	//read D_i
	for (j = 0; j < N_frames_cal; j++) {
		//read D_i
		for (i = 0; i < N_D; i++)
		{
			fscanf(calreadings, "%lg, %lg, %lg", &x, &y, &z);
			temp << x, y, z;
			D.push_back(temp);
		}
		//read A_i
		for (i = 0; i < N_A; i++)
		{
			fscanf(calreadings, "%lg, %lg, %lg", &x, &y, &z);
			temp << x, y, z;
			A.push_back(temp);
		}
		//read C_i
		for (i = 0; i < N_C; i++)
		{
			fscanf(calreadings, "%lg, %lg, %lg", &x, &y, &z);
			temp << x, y, z;
			C.push_back(temp);
		}
	}

	fclose(calreadings);
#pragma endregion

#pragma region read empivot file
	FILE* empivot;
	strcpy(read, ("..\\INPUT\\" + cas + "-empivot.txt").c_str());
	empivot = fopen(read, "r");
	fscanf(empivot, "%d, %d, %s", &N_G, &N_frames_empivot, s);
	cout << "reading:" << s << endl;
	//read
	for (j = 0; j < N_frames_empivot; j++) {
		//read G_i
		for (i = 0; i < N_G; i++)
		{
			fscanf(empivot, "%lg, %lg, %lg", &x, &y, &z);
			temp << x, y, z;
			G.push_back(temp);
		}
	}
	fclose(empivot);
#pragma endregion

#pragma region read emfiducials file
	FILE* emfiducials;
	strcpy(read, ("..\\INPUT\\" + cas + "-em-fiducialss.txt").c_str());
	emfiducials = fopen(read, "r");
	fscanf(emfiducials, "%d, %d, %s", &N_G, &N_B, s);
	cout << "reading:" << s << endl;
	//read
	for (j = 0; j < N_B; j++) {
		//read G_fiducial
		for (i = 0; i < N_G; i++)
		{
			fscanf(emfiducials, "%lg, %lg, %lg", &x, &y, &z);
			temp << x, y, z;
			G_fidu.push_back(temp);
		}
	}
	cout << "G_fidu size:" << G_fidu.size() << " N_G*N_B: " << N_G * N_B << endl;
	fclose(emfiducials);
#pragma endregion

#pragma region read ctfiducials file
	FILE* ctfiducials;
	strcpy(read, ("..\\INPUT\\" + cas + "-ct-fiducials.txt").c_str());
	ctfiducials = fopen(read, "r");
	fscanf(ctfiducials, "%d, %s", &N_B, s);
	cout << "reading:" << s << endl;
	//read
	for (i = 0; i < N_B; i++)
	{
		fscanf(ctfiducials, "%lg, %lg, %lg", &x, &y, &z);
		temp << x, y, z;
		b.push_back(temp);
	}
	fclose(ctfiducials);
#pragma endregion

#pragma region read emnav file
	FILE* emnav;
	strcpy(read, ("..\\INPUT\\" + cas + "-EM-nav.txt").c_str());
	emnav = fopen(read, "r");
	fscanf(emnav, "%d, %d, %s", &N_G, &N_frames_nav, s);
	cout << "reading:" << s << endl;
	//read
	for (j = 0; j < N_frames_nav; j++) {
		//read G_nav_i
		for (i = 0; i < N_G; i++)
		{
			fscanf(emnav, "%lg, %lg, %lg", &x, &y, &z);
			temp << x, y, z;
			G_nav.push_back(temp);
		}
	}
	fclose(emnav);
#pragma endregion

#pragma endregion

#pragma region Q1 Compute C_expected
	/*****Q1 Compute C_expected*****/

	//Compute a frame F_D//
	vector<F> F_D;
	for (i = 0; i < N_frames_cal; i++) {
		vector<Vector3d>::const_iterator First = D.begin() + floor(i * N_D);
		vector<Vector3d>::const_iterator Second = D.begin() + floor(i * N_D) + N_D;
		vector<Vector3d> Di(First, Second);
		F F_temp = Point3D_Sets_Registration(d, Di);
		F_D.push_back(F_temp);
	}

	//Compute a frame F_A//
	vector<F> F_A;
	for (i = 0; i < N_frames_cal; i++) {
		vector<Vector3d>::const_iterator First = A.begin() + floor(i * N_A);
		vector<Vector3d>::const_iterator Second = A.begin() + floor(i * N_A) + N_A;
		vector<Vector3d> Ai(First, Second);
		F F_temp = Point3D_Sets_Registration(a, Ai);
		F_A.push_back(F_temp);
		//cout << "F_A_R" << i << " = " << endl << F_temp.R << endl;
		//cout << "F_A_p" << i << " = " << endl << F_temp.p << endl;
	}

	//Compute a Ci_expected//
	vector<Vector3d> C_expected;
	for (i = 0; i < N_frames_cal; i++) {
		for (j = 0; j < N_C; j++) {
			F F_D_inv = F_D[i].inv();
			//cout << "F_D_inv" << i << " = " << endl << F_D_inv.R << endl;
			//cout << "F_D_inv" << i << " = " << endl << F_D_inv.p << endl;
			Vector3d C_temp = F_D_inv * (F_A[i] * c[j]);
			C_expected.push_back(C_temp);
		}
	}

	//test
	/*for (i = 0; i < N_frames_cal; i++) {
		vector<Vector3d>::const_iterator FirstD = D.begin() + floor(i * N_D);
		vector<Vector3d>::const_iterator SecondD = D.begin() + floor(i * N_D) + N_D;
		vector<Vector3d> Di(FirstD, SecondD);
		for (j = 0; j < N_D; j++) {
			cout << Di[j].transpose() << endl;
		}
		vector<Vector3d>::const_iterator FirstA = A.begin() + floor(i * N_A);
		vector<Vector3d>::const_iterator SecondA = A.begin() + floor(i * N_A) + N_A;
		vector<Vector3d> Ai(FirstA, SecondA);
		for (j = 0; j < N_A; j++) {
			cout << Ai[j].transpose() << endl;
		}
		vector<Vector3d>::const_iterator FirstC = C.begin() + floor(i * N_C);
		vector<Vector3d>::const_iterator SecondC = C.begin() + floor(i * N_C) + N_C;
		vector<Vector3d> Ci(FirstC, SecondC);
		for (j = 0; j < N_C; j++) {
			cout << Ci[j].transpose() << endl;
		}
	}*/
	/*for (j = 0; j < N_D; j++) {
		cout << d[j].transpose() << endl;
	}
	for (j = 0; j < N_A; j++) {
		cout << a[j].transpose() << endl;
	}
	for (j = 0; j < N_C; j++) {
		cout << c[j].transpose() << endl;
	}*/
#pragma endregion


#pragma region Q2 Distortion correction function

	vector<Vector3d> p = C_expected;   // Known 3D "ground truth"
	vector<Vector3d> q = C;             // Values returned by navigational sensor

	/*Determine a bounding box to scale q values*/
	int N = 5; // Using 5 degree Bernstein polynomials
	MatrixXd us_matrix = ScaleToBox(N, N_frames_cal, N_C, q);
	//cout << "us_matrix = " << endl << us_matrix << endl;

	/*Set up and solve the least squares problem*/
	//Construct a "tensor form" interpolation polynomial

	MatrixXd F_us;
	F_us = Eigen::Matrix<double, Dynamic, 216>();
	F_us.resize(N_frames_cal * N_C, 216);
	for (i = 0; i < N_frames_cal * N_C; i++) {
		MatrixXd temp = Fijk(5, us_matrix(i, 0), us_matrix(i, 1), us_matrix(i, 2));    //Compute Fijk for each us
		//cout << "temp" << temp << endl;
		F_us.block<1, 216>(i, 0) = temp;
	}
	//cout << F_us.size();

	//Construct ps_matrix
	/*Do not scale p to bounding box*/
	MatrixXd ps_matrix(N_frames_cal* N_C, 3);
	for (i = 0; i < N_frames_cal; i++) {
		vector<Vector3d>::const_iterator First = p.begin() + floor(i * N_C);
		vector<Vector3d>::const_iterator Second = p.begin() + floor(i * N_C) + N_C;
		vector<Vector3d> pi(First, Second);
		for (j = 0; j < N_C; j++) {
			temp = pi[j];
			int n = i * N_C + j;     //row number of us_mtrix
			ps_matrix(n, 0) = temp[0];
			ps_matrix(n, 1) = temp[1];
			ps_matrix(n, 2) = temp[2];
		}
	}
	//MatrixXd ps_matrix = ScaleToBox(N, N_frames, N_C, p);
	//cout << ps_matrix << endl;


	//Compute c_matrix by the least squares problem
	//SVD decomposition approach to solve the least square problem A*p=B, here A=F_us, B=ps_matrix
	MatrixXd c_matrix = F_us.bdcSvd(ComputeThinU | ComputeThinV).solve(ps_matrix);
	//MatrixXd c_matrix = (F_us.transpose() * F_us).ldlt().solve(F_us.transpose() * ps_matrix);
	//cout << c_matrix << endl;

	/*c_matrix is used in the correction function: */
	vector<Vector3d> p1 = CorrectDistortion(N, N_frames_cal, N_C, q, c_matrix);
	/*for (i = 0; i < N_frames_cal * N_C; i++) {
		cout << "pcor:" << p1[i].transpose() << endl;
		cout << "p:" << p[i].transpose() << endl;
		cout << "q" << q[i].transpose() << endl;
	}*/

#pragma endregion

#pragma region Q4 Calculate f using corrected G
	//	Q4
	vector<Vector3d> G_corrected;
	vector<Vector3d> G_fidu_corrected;
	vector<Vector3d> G_nav_corrected;
	vector<Vector3d> g; //g[i] time-invariant
	VectorXd p_G(6);
	//	correct G's distortion
	G_corrected = CorrectDistortion(N, N_frames_empivot, N_G, G, c_matrix);  //empivot frame should be 12, N_G=6
	G_fidu_corrected = CorrectDistortion(N, N_B, N_G, G_fidu, c_matrix);
	G_nav_corrected = CorrectDistortion(N, N_frames_nav, N_G, G_nav, c_matrix);
	//	G_corrected = G;
	//compute p_tip
	p_G = PCalibrationPA1(G_corrected, N_G, N_frames_empivot, g); //p_G contains ptip (0-2) and pdimple (3-5)
	//important note  g would change after above line

	const Vector3d p_tip = p_G.block<3, 1>(0, 0);  //p_tip fixed

//calculate fi
//fi=F_ptr_i*p_tip
	vector<Vector3d> f;  //container for fis with size of fiducial numbers N_B
	for (i = 0;i < N_B;i++) {  //fi for each fiducial point, N_B=6
		vector<Vector3d>::const_iterator First = G_fidu_corrected.begin() + floor(i * N_G);
		vector<Vector3d>::const_iterator Second = G_fidu_corrected.begin() + floor(i * N_G) + N_G;
		vector<Vector3d> G_fidu_i(First, Second);
		F Fi = Point3D_Sets_Registration(g, G_fidu_i);  //F_ptr_i
		f.push_back(Fi * p_tip);
	}
	cout << "f size should be 6" << f.size() << endl;
#pragma endregion

#pragma region Q5 Calculate F_reg
	//Q5
	// registration using point cloud bi and fi (each have N_B points)
	// F_reg * b = f
	F F_reg = Point3D_Sets_Registration(b, f);
	cout << "F_reg.p = " << endl << F_reg.p << endl;
	cout << "F_reg.R = " << endl << F_reg.R << endl;
#pragma endregion

#pragma region Q6 Calculate navigation points under CT scheme
	//Q6
	//F_nav_i=regis(g,G_nav)
	//f_nav_i=F_nav_i*p_tip
	//v_i=F_reg.inv()*f_nav_i
	vector<Vector3d> v_nav;
	for (i = 0;i < N_frames_nav;i++) {  //fi for each nav point, N_frame_nav=4
		vector<Vector3d>::const_iterator First = G_nav_corrected.begin() + floor(i * N_G);
		vector<Vector3d>::const_iterator Second = G_nav_corrected.begin() + floor(i * N_G) + N_G;
		vector<Vector3d> G_nav_i(First, Second);
		F F_nav_i = Point3D_Sets_Registration(g, G_nav_i);
		Vector3d f_nav_i = F_nav_i * p_tip;
		v_nav.push_back((F_reg.inv()) * f_nav_i);
	}
	for (i = 0; i < N_frames_nav; i++) {
		cout << "v_nav:" << v_nav[i].transpose() << endl;
	}

#pragma endregion

	/*****Output*****/
	ofstream output("..\\OUTPUT\\SZ&QW-" + cas + "-OUTPUT-2.txt");

	//Write data into the "SZ&QW-OUTPUT-2.txt" file
	output << N_frames_nav << "," << "     " << "SZ&QW-" + cas + "-OUTPUT-2.txt" << endl;

	for (i = 0; i < N_frames_nav; i++) {
		output << v_nav[i][0] << "," << "     " << v_nav[i][1] << "," << "     " << v_nav[i][2] << endl;
	}
	output.close();


}