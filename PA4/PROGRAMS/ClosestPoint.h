#include <iostream>
#include <Eigen\Dense>
#include <stdio.h>
#include "vector"
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"
#include "ProjectOnSegment.h"

using namespace Eigen;
using namespace std;

Vector3d ClosestPoint(Vector3d d, Vector3d v1, Vector3d v2, Vector3d v3);
