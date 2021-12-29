#include <iostream>
#include <Eigen\Dense>
#include <stdio.h>
#include "vector"
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"

using namespace Eigen;
using namespace std;

Vector3d ProjectOnSegment(const Vector3d& c, const Vector3d& v1, const Vector3d& v2);
