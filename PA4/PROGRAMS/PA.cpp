// PA.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"
#include "3Dpoint_sets_registration.h"
#include "vector"
#include "stdio.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <Eigen\Dense>
#include "matching.h"
#include "matching_boundingbox.h"
#include <iomanip>

using namespace std;

int main()
{
#pragma region Define problem set variables
	//Define problem set variables
	int N_A, N_B, N_S, N_samps, N_ver, N_tri;
	vector<Vector3d> a;  //marker position under body coordinate
	vector<Vector3d> b;
	vector<Vector3d> Ver;  //positions of surface vertices under CT frame
	vector<VectorXi> Tri;  //info of triangle meshes, note: class integer
	Vector3d a_tip;  //tip position under body coordinate
	Vector3d b_tip;
#pragma endregion

#pragma region Define global assistant variables
	int i, j, k, n, m, start, end;
	double x, y, z, con_sigma, rel_sigma;
	char s[60];
	string cas;
	Vector3d temp;
	char read[60];
#pragma endregion
	
	cout << "Please specify convergence criteria of ICP process." << endl;
	cout << "sigma=(e.g.0.001)" ;
	cin >> con_sigma;
	cout << "sigma_relative=(e.g.0.95)";
	cin >> rel_sigma;


	cout << "Please type in start case. (e.g. 0): (pa4-debug-a=0, pa4-debug-b=1,...,pa4-unknown-K=9):";
	cin >> start;
	cout << "Please type in end case. (e.g. 9): (pa4-debug-a=0, pa4-debug-b=1,...,pa4-unknown-K=9):";
	cin >> end;

#pragma region read bodyA file
	//Read bodyA file
	FILE* bodyA;
	bodyA = fopen("..\\INPUT\\Problem4-BodyA.txt", "r");
	fscanf(bodyA, "%d %s", &N_A, s);
	cout << "reading:" << s << endl;
	//read BodyA LED markers
	for (i = 0; i < N_A; i++)
	{
		fscanf(bodyA, "%lg %lg %lg", &x, &y, &z);
		temp << x, y, z;
		a.push_back(temp);
		//cout << temp << endl;
	}
	fscanf(bodyA, "%lg %lg %lg", &x, &y, &z);
	a_tip << x, y, z;
	fclose(bodyA);
#pragma endregion

#pragma region read bodyB file
	//Read bodyA file
	FILE* bodyB;
	bodyB = fopen("..\\INPUT\\Problem4-BodyB.txt", "r");
	fscanf(bodyA, "%d %s", &N_B, s);
	cout << "reading:" << s << endl;
	//read BodyA LED markers
	for (i = 0; i < N_B; i++)
	{
		fscanf(bodyB, "%lg %lg %lg", &x, &y, &z);
		temp << x, y, z;
		b.push_back(temp);
		//cout << temp << endl;
	}
	fscanf(bodyB, "%lg %lg %lg", &x, &y, &z);
	b_tip << x, y, z;
	fclose(bodyB);
#pragma endregion

#pragma region read surface mesh file
	FILE* surf;
	surf = fopen("..\\INPUT\\Problem4MeshFile.sur", "r");
	cout << "reading: Problem4MeshFile.sur" << endl;
	fscanf(surf, "%d", &N_ver);

	for (i = 0; i < N_ver; i++)
	{
		fscanf(surf, "%lg %lg %lg", &x, &y, &z);
		temp << x, y, z;
		//cout << temp << endl;
		//temp[0] = round(temp[0] * 100) / 100;
		//temp[1] = round(temp[1] * 100) / 100;
		//temp[2] = round(temp[2] * 100) / 100;
		Ver.push_back(temp);
	}

	fscanf(surf, "%d", &N_tri);

	for (i = 0; i < N_tri; i++)
	{
		int t1, t2, t3, t4, t5, t6;
		VectorXi temp6(6);
		fscanf(surf, "%d %d %d %d %d %d", &t1, &t2, &t3, &t4, &t5, &t6);
		temp6 << t1, t2, t3, t4, t5, t6;
		//cout << temp6 << endl;
		Tri.push_back(temp6);
	}

	fclose(surf);
#pragma endregion

	/**/vector<string> stingvec;
	stingvec.push_back("PA4-A-Debug");
	stingvec.push_back("PA4-B-Debug");
	stingvec.push_back("PA4-C-Debug");
	stingvec.push_back("PA4-D-Debug");
	stingvec.push_back("PA4-E-Debug");
	stingvec.push_back("PA4-F-Debug");
	stingvec.push_back("PA4-G-Unknown");
	stingvec.push_back("PA4-H-Unknown");
	stingvec.push_back("PA4-J-Unknown");
	stingvec.push_back("PA4-K-Unknown");
	for (m = start; m <= end; m++) {
		vector<Vector3d> A;  //marker position under tracker coordinate
		vector<Vector3d> B;
		cas = stingvec[m];
		cout << cas << endl;
		/*cout << "Please type in case num. (e.g. PA3-A-Debug, etc..):";
		cin >> cas;
		cout << "Processing: " << cas << endl;*/

#pragma region read samplereadings file
		//Read samplereadings file
		FILE* samplereadings;
		strcpy(read, ("..\\INPUT\\" + cas + "-SampleReadingsTest.txt").c_str());
		samplereadings = fopen(read, "r");
		double xx;
		fscanf(samplereadings, "%d, %d, %s %d", &N_S, &N_samps, s, &xx);
		cout << "reading:" << s << endl;

		for (j = 0; j < N_samps; j++) {
			//read A_i
			for (i = 0; i < N_A; i++)
			{
				fscanf(samplereadings, "%lg, %lg, %lg", &x, &y, &z);
				temp << x, y, z;
				//cout << temp << endl;
				A.push_back(temp);
			}
			//read B_i
			for (i = 0; i < N_B; i++)
			{
				fscanf(samplereadings, "%lg, %lg, %lg", &x, &y, &z);
				temp << x, y, z;
				//cout << temp << endl;
				B.push_back(temp);
			}
			//read dummy markers
			for (i = 0; i < (N_S - N_A - N_B); i++)
			{
				fscanf(samplereadings, "%lg, %lg, %lg", &x, &y, &z);
			}
		}

		fclose(samplereadings);
#pragma endregion

#pragma region Frame registration
		vector<F> F_A;
		for (i = 0; i < N_samps; i++) {
			vector<Vector3d>::const_iterator First = A.begin() + floor(i * N_A);
			vector<Vector3d>::const_iterator Second = A.begin() + floor(i * N_A) + N_A;
			vector<Vector3d> Ai(First, Second);
			F F_temp = Point3D_Sets_Registration(a, Ai);
			F_A.push_back(F_temp);
		}

		vector<F> F_B;
		for (i = 0; i < N_samps; i++) {
			vector<Vector3d>::const_iterator First = B.begin() + floor(i * N_B);
			vector<Vector3d>::const_iterator Second = B.begin() + floor(i * N_B) + N_B;
			vector<Vector3d> Bi(First, Second);
			F F_temp = Point3D_Sets_Registration(b, Bi);
			F_B.push_back(F_temp);
		}
#pragma endregion

		//cout << Ver[900] << endl;
		//cout << Tri[900] << endl;
		vector<Vector3d> d;
		for (k = 0; k < N_samps; k++) {
			Vector3d d_temp = (F_B[k].inv()) * F_A[k] * a_tip;
			d.push_back(d_temp);
		}

		Vector3d zero;
		zero << 0, 0, 0;
		F T(MatrixXd::Identity(3, 3), zero);
		//		Matrix3d R_test;
		//		Vector3d p_test;
		//		R_test << -1, 0, 0,
		//			0, -1, 0,
		//			0, 0, 1;
		//		p_test << -1, 1, 2;
		//		F T(R_test, p_test);
		vector<Vector3d> c;
		double yita;  // uninitialized
		n = 0;

		double sigma_sum, epsilon_sum, epsilon_max, sigma, epsilon;
		double ratio_epsilon = 0;
		double ratio_sigma = 0;
		while ((n == 0 || sigma > con_sigma) && ratio_sigma < rel_sigma) {   //Set Convergence Criteria here
			vector<Vector3d> C;
			vector<Vector3d> D;
			Vector3d ek;
			c = matching_boundingbox(N_tri, T, d, Ver, Tri);
			for (i = 0;i < d.size();i++) {
				if (n == 0 || ((T*d[i] - c[i]).norm()) < yita) {
					C.push_back(c[i]);
					D.push_back(d[i]);
				}
			}

			n = n + 1;
			T = Point3D_Sets_Registration(D, C);  //update Tn+1
			cout << T.R << endl;
			cout << T.p << endl;

			// convergence parameter calculation
			sigma_sum = 0;
			epsilon_sum = 0;
			epsilon_max = 0;
			for (i = 0;i < C.size();i++) {
				ek = T * D[i] - C[i];
				sigma_sum = sigma_sum + dotp(ek, ek);
				epsilon_sum = epsilon_sum + ek.norm();
				if (ek.norm() > epsilon_max) {
					epsilon_max = ek.norm();
				}
			}
			if (n > 1) {
				ratio_epsilon = (epsilon_sum / C.size()) / epsilon; //en/en-1
				ratio_sigma = (sigma_sum / C.size()) / sigma;
			}
			sigma = sigma_sum / C.size();
			epsilon = epsilon_sum / C.size();
			yita = 3 * epsilon;  //update yitan+1
			cout << "n: " << n << endl;
			cout << "sigma, epsilon: " << sigma << ", " << epsilon << endl;
			cout << "iter ratio of sigma, epsilon: " << ratio_sigma << ", " << ratio_epsilon << endl;
		}


		vector<double> Dis;             //The distance of d and c
		F F_reg = T;
		for (i = 0; i < d.size(); i++) {
			double dis = (F_reg * d[i] - c[i]).norm();
			Dis.push_back(dis);
			//cout << d[i] << endl;
			//cout << index[i]<<endl;
			//cout << Ver[index[i]] << endl << endl;
			/*for (j = 0; j < Ver.size(); j++) {
				if (d[i][0] == Ver[j][0] || d[i][1] == Ver[j][1] || d[i][2] == Ver[j][2]) {
					cout << d[i] << endl;
					cout << Ver[j] << endl << endl;
				}
			}*/
		}

#pragma endregion

		/*****Output*****/
		ofstream output("..\\OUTPUT\\SZ&QW-" + cas + "-Output.txt");

		//Write data into the "SZ&QW-OUTPUT-4.txt" file
		output << N_samps << "," << "     " << "SZ&QW-" + cas + "-Output.txt" << endl;

		for (i = 0; i < N_samps; i++) {
			output.setf(ios::fixed);
			output << setw(6) << setprecision(2) << (F_reg*d[i])[0] << setw(13) << setprecision(2) << (F_reg*d[i])[1] << setw(13) << setprecision(2) << (F_reg*d[i])[2] << setw(16) << setprecision(2) << c[i][0] << setw(13) << setprecision(2) << c[i][1] << setw(13) << setprecision(2) << c[i][2] << setw(13) << setprecision(3) << Dis[i] << endl;
			//			output << (F_reg*d[i])[0] << "  " << (F_reg*d[i])[1] << "  " << (F_reg*d[i])[2] << "  " << c[i][0] << "  " << c[i][1] << "  " << c[i][2] << "  " << Dis[i] << endl;
		}
		output.close();

#pragma endregion
		/*****Test the correctness****
		int N_Samples;
		int N_Samples_given;
		double ze, x1, y1, z1, dis_norm;
		char str[50];
		char str1[50];
		double error = 0.;
		double error_ave = 0.;
		double error_i = 0.;

		//Open the file of output we computed above
		FILE* output_;
		strcpy(read, ("..\\OUTPUT\\SZ&QW-" + cas + "-Output.txt").c_str());
		output_ = fopen(read, "r");
		fscanf(output_, "%d, %s", &N_Samples, str);
		//Comupute the number of rows in the outputfile
		int num = N_Samples;
		MatrixXd output_matrix(num, 7);
		//Make a matix that contains all of the data in the output file
		for (i = 0; i < num; i++) {
			fscanf(output_, "%lg, %lg, %lg, %lg, %lg, %lg, %lg", &x, &y, &z, &x1, &y1, &z1, &dis_norm);
			output_matrix(i, 0) = x;
			output_matrix(i, 1) = y;
			output_matrix(i, 2) = z;
			output_matrix(i, 3) = x1;
			output_matrix(i, 4) = y1;
			output_matrix(i, 5) = z1;
			output_matrix(i, 6) = dis_norm;
		}
		fclose(output_);
		cout << output_matrix << endl;
		//Open the file of output given by the class file
		FILE* output_test;
		strcpy(read, ("..\\INPUT\\" + cas + "-Output.txt").c_str());
		output_test = fopen(read, "r");
		fscanf(output_test, "%d,  %s, %d", &N_Samples_given, str1, &ze);
		//Comupute the number of rows in the outputfile
		num = N_Samples_given;
		//Make a test data matix that contains all of the data in the output file
		MatrixXd output_test_matrix(num, 7);
		for (i = 0; i < num; i++) {
			fscanf(output_test, "%lg, %lg, %lg, %lg, %lg, %lg, %lg", &x, &y, &z, &x1, &y1, &z1, &dis_norm);
			output_test_matrix(i, 0) = x;
			output_test_matrix(i, 1) = y;
			output_test_matrix(i, 2) = z;
			output_test_matrix(i, 3) = x1;
			output_test_matrix(i, 4) = y1;
			output_test_matrix(i, 5) = z1;
			output_test_matrix(i, 6) = dis_norm;
		}
		fclose(output_test);

		cout << output_test_matrix << endl;
		cout << output_matrix << endl;
		//Compare the data from both matrixs and compute the average error
		for (i = 0; i < num; i++) {
			for (j = 0; j < 7; j++) {
				y = output_test_matrix(i, j);
				x = output_matrix(i, j);
				error_i = abs(y - x);                //compute the error of each data
				error += error_i;
			}
		}
		error_ave = error / num;               //compute the average error
		cout << "The average error of the computed output is:  " << error_ave; */
	}
}

