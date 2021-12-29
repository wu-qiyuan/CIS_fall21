Source files:
--<vector>:Provides vector used in the data collection.
--<stdlib.h>:Allow us to operate memory partition.
--<iostream>:Allow us to use cin and cout.
--<fstream>:Helps to read and write files.
--<stdio.h>:Help output in a neat format.
--<iomanip>：Help output in a neat format.
--<Eigen\Dense>:Provides most of the definations and operators of matrix and point, as well as the SVD decomposition and QR decomposition.
--CatesianMathBasic.h
   Vector3d crossp(Vector3d a, Vector3d b)
   double dotp(Vector3d a, Vector3d b)
   Matrix3d invR(Matrix3d R)
   Matrix3d Rot3(Vector3d w, double theta)
--F.h
  Class members: Rotation matrix R and translation vector p.
  Class functions: F operates F; F operates point; Inverse of F.
--3Dpoint_sets_registration.h
  Function: F Point3D_Sets_Registration(vector<Vector3d> &a, vector<Vector3d> &b).
--pivot.h
  Function: VectorXd pivot(std::vector<F> &Fdata).
--matching.h: 
  Function: vector<int> matching(F F_guess, const vector<Vector3d> &q, const vector<Vector3d> &c)
--ProjectOnSegment.h
  Function: Vector3d ProjectOnSegment(const Vector3d& c, const Vector3d& v1, const Vector3d& v2)

