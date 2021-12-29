#pragma once
#include <Eigen\Dense>
#include "pch.h"

class F
{
public:
	Eigen::Matrix3d R;
	Eigen::Vector3d p;

	F();

	F(Eigen::Matrix3d Ri, Eigen::Vector3d pi); //overloading for constructor

	~F();

	F operator*(const F& b); // F operates F

	Eigen::Vector3d operator*(const Eigen::Vector3d& b);  // F operates point

	F inv();

};
