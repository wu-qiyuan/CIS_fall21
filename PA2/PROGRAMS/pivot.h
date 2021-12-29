//  Solve the equation p_pivot = R_G[k] * t + p_G[k]
//  Input F[]=[R_G,p_G][];
//  Output p = [t;p_pivot]

#include <Eigen\Dense>
#include "pch.h"
#include "F.h"
#include "CatesianMathBasic.h"
#include "vector"


VectorXd pivot(std::vector<F> &Fdata);