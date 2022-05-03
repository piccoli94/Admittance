/* ========================= ICR_Simulation ================ */
/*         All right this code is reserved for the ICAROS
* Desrcption:
This is a controller based on a region control law

Author: Piccoli Danilo
Email: piccoli94@gmail.com
Date: 21/07/21

Revision History:
--------------------------------
Version V1.0: Initial Version

=============================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include "regressor.cpp"
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
Matrix<double,1,413> Y_d;
Matrix<double,7,1> q;
Matrix<double,7,1> dq;
Matrix<double,7,1> dqr;
Matrix<double,7,1> ddqr;
Y_d=regressor(ddqr,dqr,dq,q);


std::cout << "Y_b work" << std::endl;



cout <<Y_d<< endl;
}





