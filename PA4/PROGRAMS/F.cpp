#include "pch.h"
#include "F.h"
#include <Eigen\Dense>


F::F()
{
	this->R << Eigen::MatrixXd::Identity(3, 3);

	Eigen::Vector3d p0(0, 0, 0);
	this->p << p0;
}


F::~F()
{
}


F::F(Eigen::Matrix3d Ri, Eigen::Vector3d pi)
{
	R = Ri;
	p = pi;
}

F F::operator*(const F& b)
{
	F Result;
	Result.R = this->R * b.R;
	Result.p = this->R * b.p + this->p;
	return Result;
}

Eigen::Vector3d F::operator*(const Eigen::Vector3d& b)
{
	Eigen::Vector3d Result;
	Result = this->R * b + this->p;
	return Result;
}

F F::inv()
{
	F Result;
	Result.R = this->R.transpose();
	Result.p = -this->R.transpose()*this->p;
	return Result;
}