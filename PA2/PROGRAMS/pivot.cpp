//  Solve the equation p_pivot = R_G[k] * t + p_G[k]
//  Input F[]=[R_G,p_G][];
//  Output p = [t;p_pivot]

#include <Eigen\Dense>
#include "vector"
#include "pch.h"
#include "F.h"
#include <iostream>

using namespace Eigen;

VectorXd pivot(std::vector<F> &Fdata)  //read data set of Fi = [Ri,pi]
{
	MatrixXd Func;
	VectorXd p(6);  //[pt;ppivot]
	MatrixXd Jfc;  //combined Jacobian matrix
	VectorXd pc;  //combined right side term(RST) (combined p0-pk)
	int i;

//	Func = Fdata[i].R*p.block<3, 1>(0, 0) - p.block<3, 1>(3, 0); this is expression of f

// Dynamic size assignment to combined Jacobian matrix and combined RST
	Jfc = Eigen::Matrix<double, Dynamic, Dynamic>();
	Jfc.resize(Fdata.size()*3,6);
	pc = Eigen::Matrix<double, Dynamic, 1>();
	pc.resize(Fdata.size() * 3, 1);

// Constructing combined Jacobian matrix (3i*6)
	for (i = 0;i < Fdata.size();i++) {
		Jfc.block<3, 6>(i * 3, 0) << Fdata[i].R, -MatrixXd::Identity(3, 3); //3*6 for each i
	}

// Constructing combined RST (3i*1)
	for (i = 0;i < Fdata.size();i++) {
		pc.block<3, 1>(i * 3, 0) << - Fdata[i].p; //3*1 for each i, notify minus p here
	}

// SVD decomposition approach to solve the least square problem A*p=B, here A=Jfc, B=pc
	p = Jfc.bdcSvd(ComputeThinU | ComputeThinV).solve(pc);

// QR decomposition approach to solve the least square problem A*p=B, here A=Jfc, B=pc
//	p = Jfc.colPivHouseholderQr().solve(pc);

	return p;  //return p as [pt;ppivot]
}