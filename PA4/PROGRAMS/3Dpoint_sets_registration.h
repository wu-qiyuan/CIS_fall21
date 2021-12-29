// CIS_Assignment1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

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

F Point3D_Sets_Registration(vector<Vector3d> &a, vector<Vector3d> &b);