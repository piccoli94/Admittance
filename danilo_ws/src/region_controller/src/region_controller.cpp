//#include "region_controller.h"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "regressor.cpp"

//#include "sensor_msgs/JointState.h"
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
using namespace Eigen;
using namespace std;
class REGION_CONTROLLER
{
private:
    Matrix<double, 7, 1> q;
    Matrix<double, 7, 1> dq;
    Matrix<double, 3, 1> xdes;
    Matrix<double, 3, 1> dxdes;
    Matrix<double, 3, 1> x_i;
    Matrix<double, 59, 1> theta_cap;
    double a_h;
    double a_r;
    double a_t;
    double alpha;
    double k_max;
    double ts;
    Matrix<double, 7, 7> K_s;
    Matrix<double, 59, 59> L;
    Matrix<double, 3, 1> x_e;
    Matrix<double, 3, 1> x_e_dot;
    Matrix<double, 3, 1> Delta_x;
    double Delta_x_norm;
    std::string region;
    double w;
    double dw_part;
    Matrix<double,3,3> A;
    Matrix<double, 3, 1> x_d;
    Matrix<double, 3, 1> delta_x;
    double k_p;
    Matrix<double,3,7>J;
    Matrix<double,7,3> pinv_J;
    double k_pinv_J;
    Matrix<double, 7, 7> M;
    Matrix<double, 7, 7> I7=Matrix<double, 7, 7>::Identity();
    Matrix<double, 3, 3> I3=Matrix<double, 3, 3>::Identity();;
    Matrix<double, 7, 7> M_inverse;
    Matrix<double, 3, 1> x_f_dot;
    Matrix<double, 7, 1> q_r_dot;
    Matrix<double, 7, 1> dqr_old;
    Matrix<double, 7, 1> s;
    Matrix<double, 7, 1> q_r_dot_dot;
    Matrix<double, 7, 59> Y_d;
    Matrix<double, 59, 1> theta_cap_dot;
    Matrix<double, 7, 1> tau_m_a;
    Matrix<double, 7, 1> tau_m_b;
    Matrix<double, 7, 1> tau_m_c;
    Matrix<double, 7, 1> tau_m;
    double norm_Delta_x;
    Matrix<double,3,1> dw_dx;
    int dof;
    Matrix<double, 3, 3> Jtemp;
    Matrix<double, 6, 7> J_dof_6;

    	Matrix<double,1,413> Y_d_temp;
    	Matrix<double, 4, 4> Tbe;
    	double db=0.0; 
    	double d0=0.34; 
    	double d3=0.40; 
   	double d5=0.40; 
    	double d7=0.126;




public:

void forward_kinematics()
{

    Tbe(0,0)=cos(q(6))*(sin(q(5))*(sin(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))+cos(q(0))*cos(q(3))*sin(q(1)))+cos(q(5))*(cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))-cos(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(2))*sin(q(0))+cos(q(0))*cos(q(1))*sin(q(2)))))-sin(q(6))*(sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))-cos(q(0))*sin(q(1))*sin(q(3)))-cos(q(4))*(cos(q(2))*sin(q(0))+cos(q(0))*cos(q(1))*sin(q(2))));
	Tbe(0,1)=-sin(q(6))*(sin(q(5))*(sin(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))+cos(q(0))*cos(q(3))*sin(q(1)))+cos(q(5))*(cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))-cos(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(2))*sin(q(0))+cos(q(0))*cos(q(1))*sin(q(2)))))-cos(q(6))*(sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))-cos(q(0))*sin(q(1))*sin(q(3)))-cos(q(4))*(cos(q(2))*sin(q(0))+cos(q(0))*cos(q(1))*sin(q(2))));
	Tbe(0,2)=cos(q(5))*(sin(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))+cos(q(0))*cos(q(3))*sin(q(1)))-sin(q(5))*(cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))-cos(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(2))*sin(q(0))+cos(q(0))*cos(q(1))*sin(q(2))));
	Tbe(0,3)=d5*(sin(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))+cos(q(0))*cos(q(3))*sin(q(1)))+d7*(cos(q(5))*(sin(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))+cos(q(0))*cos(q(3))*sin(q(1)))-sin(q(5))*(cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(2))-cos(q(0))*cos(q(1))*cos(q(2)))-cos(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(2))*sin(q(0))+cos(q(0))*cos(q(1))*sin(q(2)))))+d3*cos(q(0))*sin(q(1));
	Tbe(1,0)=sin(q(6))*(sin(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))+sin(q(0))*sin(q(1))*sin(q(3)))-cos(q(4))*(cos(q(0))*cos(q(2))-cos(q(1))*sin(q(0))*sin(q(2))))-cos(q(6))*(sin(q(5))*(sin(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))-cos(q(3))*sin(q(0))*sin(q(1)))+cos(q(5))*(cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))+sin(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(0))*cos(q(2))-cos(q(1))*sin(q(0))*sin(q(2)))));   
	Tbe(1,1)=sin(q(6))*(sin(q(5))*(sin(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))-cos(q(3))*sin(q(0))*sin(q(1)))+cos(q(5))*(cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))+sin(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(0))*cos(q(2))-cos(q(1))*sin(q(0))*sin(q(2)))))+cos(q(6))*(sin(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))+sin(q(0))*sin(q(1))*sin(q(3)))-cos(q(4))*(cos(q(0))*cos(q(2))-cos(q(1))*sin(q(0))*sin(q(2))));
	Tbe(1,2)=sin(q(5))*(cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))+sin(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(0))*cos(q(2))-cos(q(1))*sin(q(0))*sin(q(2))))-cos(q(5))*(sin(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))-cos(q(3))*sin(q(0))*sin(q(1)));
	Tbe(1,3)=db-d5*(sin(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))-cos(q(3))*sin(q(0))*sin(q(1)))-d7*(cos(q(5))*(sin(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))-cos(q(3))*sin(q(0))*sin(q(1)))-sin(q(5))*(cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2))+cos(q(1))*cos(q(2))*sin(q(0)))+sin(q(0))*sin(q(1))*sin(q(3)))+sin(q(4))*(cos(q(0))*cos(q(2))-cos(q(1))*sin(q(0))*sin(q(2)))))+d3*sin(q(0))*sin(q(1));
	Tbe(2,0)=sin(q(6))*(sin(q(4))*(cos(q(1))*sin(q(3))-cos(q(2))*cos(q(3))*sin(q(1)))-cos(q(4))*sin(q(1))*sin(q(2)))-cos(q(6))*(cos(q(5))*(cos(q(4))*(cos(q(1))*sin(q(3))-cos(q(2))*cos(q(3))*sin(q(1)))+sin(q(1))*sin(q(2))*sin(q(4)))-sin(q(5))*(cos(q(1))*cos(q(3))+cos(q(2))*sin(q(1))*sin(q(3))));
	Tbe(2,1)=cos(q(6))*(sin(q(4))*(cos(q(1))*sin(q(3))-cos(q(2))*cos(q(3))*sin(q(1)))-cos(q(4))*sin(q(1))*sin(q(2)))+sin(q(6))*(cos(q(5))*(cos(q(4))*(cos(q(1))*sin(q(3))-cos(q(2))*cos(q(3))*sin(q(1)))+sin(q(1))*sin(q(2))*sin(q(4)))-sin(q(5))*(cos(q(1))*cos(q(3))+cos(q(2))*sin(q(1))*sin(q(3))));
	Tbe(2,2)=sin(q(5))*(cos(q(4))*(cos(q(1))*sin(q(3))-cos(q(2))*cos(q(3))*sin(q(1)))+sin(q(1))*sin(q(2))*sin(q(4)))+cos(q(5))*(cos(q(1))*cos(q(3))+cos(q(2))*sin(q(1))*sin(q(3)));
	Tbe(2,3)=d0+d7*(sin(q(5))*(cos(q(4))*(cos(q(1))*sin(q(3))-cos(q(2))*cos(q(3))*sin(q(1)))+sin(q(1))*sin(q(2))*sin(q(4)))+cos(q(5))*(cos(q(1))*cos(q(3))+cos(q(2))*sin(q(1))*sin(q(3))))+d5*(cos(q(1))*cos(q(3))+cos(q(2))*sin(q(1))*sin(q(3)))+d3*cos(q(1));
	Tbe(3,0)=0.0;
	Tbe(3,1)=0.0;
	Tbe(3,2)=0.0;
	Tbe(3,3)=1.0;

    x_e(0)=Tbe(0,3);
    x_e(1)=Tbe(1,3);
    x_e(2)=Tbe(2,3);
   
/*
    for(int n=0;n<7;n++){
       	_q_in->data(n)=q(n);

    }
    _fksolver->JntToCart(*_q_in, _p_out);
    x_e << _p_out.p.x(), _p_out.p.y(), _p_out.p.z();
*/

}
    //----------------------------------------------------------------------Dominant Region
void    dominant_region()
{
    norm_Delta_x = Delta_x.norm();
 std::cout << "norm_Delta_x: " << norm_Delta_x << std::endl;

    if (norm_Delta_x >= 0 && norm_Delta_x <= a_h)
    {

        region = "H-DR";
    }
    else if (norm_Delta_x > a_h && norm_Delta_x <= a_r)
    {

        region = "R-DR";
    }
    else if (norm_Delta_x > a_r && norm_Delta_x <= a_t)
    {

        region = "TS-SR";
    }
    else if (norm_Delta_x > a_t)
    {
        region = "SS-SR";
    }


}

//-------------------------------------------------------------------weitght term
void    weight_vector()
{

    if (region.compare("H-DR") == 0 || region.compare("R-DR") == 0)
    {
        w = 1;
    }
    else if (region.compare("TS-SR") == 0)
    {
            w=0.5*(1+cos((Delta_x.norm()-a_r)*M_PI/(a_t-a_r)));
    }
    else if (region.compare("SS-SR") == 0)
    {
        w = 0;
    }

    }

//---------------------------------------------------------------dw/dx_i where i is the i-th element of x_e
void    derivate_w_partial()
{
    if (region.compare("H-DR") == 0 || region.compare("R-DR") == 0)
    {
        dw_part = 0;
    }
    else if (region.compare("TS-SR") == 0)
    {
        dw_part=-0.5*sin(    ( Delta_x.norm()  - a_r ) * M_PI / ( a_t - a_r ))  * M_PI /  ( ( a_t - a_r ) * Delta_x.norm() );
    }
    else if (region.compare("SS-SR") == 0)
    {
        dw_part = 0;
    }

    }

//---------------------------------------------------------------------------Matrice di transizione A
void    transition_matrix_A()
{

    dof=xdes.size();
    
    for (int i = 0; i < dof; i++)
    {
        dw_dx(i)=dw_part*(x_e(i)-xdes(i))*x_e_dot(i);
    }
    for (int i = 0; i < dof; i++)
    {
        for (int j = 0; j < dof; j++)
        {
            A(i,j)=dw_dx(j)*xdes(i);
        }

    }

    }

//---------------------------------------------------------------------------Guadagno controllo di posizione
void    pos_stiffness_tank_large()
{

    if (region.compare("H-DR") == 0)
    {
        k_p= k_max * 0.5 * ( 1 - cos ( M_PI * delta_x.norm() /a_h ) ) ;
    }
    else if (region.compare("R-DR") == 0)
    {
        k_p= k_max;
    }
    else if (region.compare("TS-SR") == 0)
    {
        k_p= k_max * 0.5 * ( 1 + cos ( M_PI * ( delta_x.norm() - a_r ) / ( a_t - a_r ) ) );
    }
    else if (region.compare("SS-SR") == 0)
    {
        k_p=0;
    }

    }


void    pos_stiffness()
{

    if (region.compare("H-DR") == 0)
    {
        k_p= 0;
    }
    else if (region.compare("R-DR") == 0)
    {
	double a=10;
	double f_p=1 - exp( - a * ( pow ( delta_x.norm() , 2 ) - pow ( a_h ,2 ) ) );
        k_p= k_max * pow( std::max( f_p , 0.0 ), 2 )  *  exp( - a * ( pow ( delta_x.norm() , 2 ) - pow ( a_h ,2 ) ) );
    }
    else if (region.compare("TS-SR") == 0)
    {
        k_p= 0;
    }
    else if (region.compare("SS-SR") == 0)
    {
        k_p=0;
    }

    }

//----------------------------------------------------------------------------Jacobiano
void    jacobiano()
{
    J(0,0)=d5*(sin(q[3])*(cos(q[0])*sin(q[2])+cos(q[1])*cos(q[2])*sin(q[0]))-cos(q[3])*sin(q[0])*sin(q[1]))+d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2])+cos(q[1])*cos(q[2])*sin(q[0]))-cos(q[3])*sin(q[0])*sin(q[1]))-sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2])+cos(q[1])*cos(q[2])*sin(q[0]))+sin(q[0])*sin(q[1])*sin(q[3]))+sin(q[4])*(cos(q[0])*cos(q[2])-cos(q[1])*sin(q[0])*sin(q[2]))))-d3*sin(q[0])*sin(q[1]);
	J(0,1)=d3*cos(q[0])*cos(q[1])+d7*cos(q[0])*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3])-cos(q[2])*cos(q[3])*sin(q[1]))+sin(q[1])*sin(q[2])*sin(q[4]))+cos(q[5])*(cos(q[1])*cos(q[3])+cos(q[2])*sin(q[1])*sin(q[3])))+d5*cos(q[0])*(cos(q[1])*cos(q[3])+cos(q[2])*sin(q[1])*sin(q[3]));
	J(0,2)=d5*cos(q[2])*sin(q[0])*sin(q[3])+d5*cos(q[0])*cos(q[1])*sin(q[2])*sin(q[3])+d7*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[3])+d7*sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5])+d7*cos(q[0])*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3])-d7*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[4])*sin(q[5])-d7*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[5])-d7*cos(q[0])*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5]);
	J(0,3)=d5*cos(q[3])*sin(q[0])*sin(q[2])-d5*cos(q[0])*sin(q[1])*sin(q[3])-d5*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])-d7*cos(q[0])*cos(q[5])*sin(q[1])*sin(q[3])+d7*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[2])+d7*cos(q[0])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5])+d7*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[3])*sin(q[5])-d7*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[5])-d7*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[3])*sin(q[5]);
	J(0,4)=-d7*sin(q[5])*(cos(q[2])*cos(q[4])*sin(q[0])+cos(q[0])*cos(q[1])*cos(q[4])*sin(q[2])+cos(q[0])*sin(q[1])*sin(q[3])*sin(q[4])-cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4])+cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*sin(q[4]));
	J(0,5)=d7*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[3])*sin(q[5])-d7*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[4])-d7*sin(q[0])*sin(q[2])*sin(q[3])*sin(q[5])-d7*cos(q[0])*cos(q[3])*sin(q[1])*sin(q[5])-d7*cos(q[0])*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[4])+d7*cos(q[0])*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[3])-d7*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[2])+d7*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5]);
	J(0,6)=0.0;
	J(1,0)=d5*(sin(q[3])*(sin(q[0])*sin(q[2])-cos(q[0])*cos(q[1])*cos(q[2]))+cos(q[0])*cos(q[3])*sin(q[1]))+d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2])-cos(q[0])*cos(q[1])*cos(q[2]))+cos(q[0])*cos(q[3])*sin(q[1]))-sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2])-cos(q[0])*cos(q[1])*cos(q[2]))-cos(q[0])*sin(q[1])*sin(q[3]))+sin(q[4])*(cos(q[2])*sin(q[0])+cos(q[0])*cos(q[1])*sin(q[2]))))+d3*cos(q[0])*sin(q[1]);
	J(1,1)=d5*sin(q[0])*(cos(q[1])*cos(q[3])+cos(q[2])*sin(q[1])*sin(q[3]))+d3*cos(q[1])*sin(q[0])+d7*sin(q[0])*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3])-cos(q[2])*cos(q[3])*sin(q[1]))+sin(q[1])*sin(q[2])*sin(q[4]))+cos(q[5])*(cos(q[1])*cos(q[3])+cos(q[2])*sin(q[1])*sin(q[3])));
	J(1,2)=d5*cos(q[1])*sin(q[0])*sin(q[2])*sin(q[3])-d7*cos(q[0])*cos(q[2])*cos(q[5])*sin(q[3])-d5*cos(q[0])*cos(q[2])*sin(q[3])-d7*cos(q[0])*sin(q[2])*sin(q[4])*sin(q[5])+d7*cos(q[1])*cos(q[5])*sin(q[0])*sin(q[2])*sin(q[3])-d7*cos(q[1])*cos(q[2])*sin(q[0])*sin(q[4])*sin(q[5])+d7*cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5])-d7*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[5]);
	J(1,3)=d7*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[5])-d5*sin(q[0])*sin(q[1])*sin(q[3])-d5*cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0])-d7*cos(q[0])*cos(q[3])*cos(q[5])*sin(q[2])-d7*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[3])-d5*cos(q[0])*cos(q[3])*sin(q[2])-d7*cos(q[0])*cos(q[4])*sin(q[2])*sin(q[3])*sin(q[5])-d7*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[5])*sin(q[0])-d7*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[0])*sin(q[3])*sin(q[5]);
	J(1,4)=-d7*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[0])*sin(q[2])-cos(q[0])*cos(q[2])*cos(q[4])+cos(q[0])*cos(q[3])*sin(q[2])*sin(q[4])+sin(q[0])*sin(q[1])*sin(q[3])*sin(q[4])+cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[4]));
	J(1,5)=d7*cos(q[0])*cos(q[2])*cos(q[5])*sin(q[4])-d7*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[5])+d7*cos(q[0])*sin(q[2])*sin(q[3])*sin(q[5])+d7*cos(q[1])*cos(q[2])*sin(q[0])*sin(q[3])*sin(q[5])-d7*cos(q[1])*cos(q[5])*sin(q[0])*sin(q[2])*sin(q[4])+d7*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[3])+d7*cos(q[0])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2])+d7*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0]);
	J(1,6)=0.0;
	J(2,0)=0.0;
	J(2,1)=d5*cos(q[1])*cos(q[2])*sin(q[3])-d5*cos(q[3])*sin(q[1])-d3*sin(q[1])-d7*cos(q[3])*cos(q[5])*sin(q[1])+d7*cos(q[1])*cos(q[2])*cos(q[5])*sin(q[3])+d7*cos(q[1])*sin(q[2])*sin(q[4])*sin(q[5])-d7*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5])-d7*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5]);
	J(2,2)=-sin(q[1])*(d5*sin(q[2])*sin(q[3])+d7*cos(q[5])*sin(q[2])*sin(q[3])-d7*cos(q[2])*sin(q[4])*sin(q[5])-d7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5]));
	J(2,3)=d5*cos(q[2])*cos(q[3])*sin(q[1])-d5*cos(q[1])*sin(q[3])-d7*cos(q[1])*cos(q[5])*sin(q[3])+d7*cos(q[2])*cos(q[3])*cos(q[5])*sin(q[1])+d7*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[5])+d7*cos(q[2])*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]);
	J(2,4)=d7*sin(q[5])*(cos(q[4])*sin(q[1])*sin(q[2])-cos(q[1])*sin(q[3])*sin(q[4])+cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]));
	J(2,5)=d7*cos(q[1])*cos(q[4])*cos(q[5])*sin(q[3])-d7*cos(q[1])*cos(q[3])*sin(q[5])-d7*cos(q[2])*sin(q[1])*sin(q[3])*sin(q[5])+d7*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])-d7*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[1]);
	J(2,6)=0.0;
/*	J(3,0)=0.0;
	J(3,1)=-sin(q[0]);
	J(3,2)=cos(q[0])*sin(q[1]);
	J(3,3)=cos(q[2])*sin(q[0])+cos(q[0])*cos(q[1])*sin(q[2]);
	J(3,4)=sin(q[3])*(sin(q[0])*sin(q[2])-cos(q[0])*cos(q[1])*cos(q[2]))+cos(q[0])*cos(q[3])*sin(q[1]);
	J(3,5)=sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2])-cos(q[0])*cos(q[1])*cos(q[2]))-cos(q[0])*sin(q[1])*sin(q[3]))-cos(q[4])*(cos(q[2])*sin(q[0])+cos(q[0])*cos(q[1])*sin(q[2]));
	J(3,6)=cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2])-cos(q[0])*cos(q[1])*cos(q[2]))+cos(q[0])*cos(q[3])*sin(q[1]))-sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2])-cos(q[0])*cos(q[1])*cos(q[2]))-cos(q[0])*sin(q[1])*sin(q[3]))+sin(q[4])*(cos(q[2])*sin(q[0])+cos(q[0])*cos(q[1])*sin(q[2])));
	J(4,0)=0.0;
	J(4,1)=cos(q[0]);
	J(4,2)=sin(q[0])*sin(q[1]);
	J(4,3)=cos(q[1])*sin(q[0])*sin(q[2])-cos(q[0])*cos(q[2]);
	J(4,4)=cos(q[3])*sin(q[0])*sin(q[1])-sin(q[3])*(cos(q[0])*sin(q[2])+cos(q[1])*cos(q[2])*sin(q[0]));
	J(4,5)=cos(q[4])*(cos(q[0])*cos(q[2])-cos(q[1])*sin(q[0])*sin(q[2]))-sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2])+cos(q[1])*cos(q[2])*sin(q[0]))+sin(q[0])*sin(q[1])*sin(q[3]));
	J(4,6)=sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2])+cos(q[1])*cos(q[2])*sin(q[0]))+sin(q[0])*sin(q[1])*sin(q[3]))+sin(q[4])*(cos(q[0])*cos(q[2])-cos(q[1])*sin(q[0])*sin(q[2])))-cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2])+cos(q[1])*cos(q[2])*sin(q[0]))-cos(q[3])*sin(q[0])*sin(q[1]));
	J(5,0)=1.0;
	J(5,1)=0.0;
	J(5,2)=cos(q[1]);
	J(5,3)=-sin(q[1])*sin(q[2]);
	J(5,4)=cos(q[1])*cos(q[3])+cos(q[2])*sin(q[1])*sin(q[3]);
	J(5,5)=cos(q[4])*sin(q[1])*sin(q[2])-sin(q[4])*(cos(q[1])*sin(q[3])-cos(q[2])*cos(q[3])*sin(q[1]));
	J(5,6)=sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3])-cos(q[2])*cos(q[3])*sin(q[1]))+sin(q[1])*sin(q[2])*sin(q[4]))+cos(q[5])*(cos(q[1])*cos(q[3])+cos(q[2])*sin(q[1])*sin(q[3]));



_jsolver->JntToJac(*_q_in, J0);
J_dof_6=J0.data;
for(int i=0;i<3;i++){
    for(int k=0;k<7;k++){
        J(i,k)=J_dof_6(i,k);
    }
}
*/

}

//----------------------------------------------------------------------------J pseudo-inversa
void    j_pseudoinverse()
{
                                                    
Jtemp = (J * J.transpose() + I3 * k_pinv_J);
pinv_J = J.transpose() * Jtemp.inverse();

}



//----------------------------------------------------------------------------------qr_dot_dot
void    derivate_dqr()
{
   q_r_dot_dot=(q_r_dot-dqr_old)/ts;
     for (int i = 0; i < q_r_dot_dot.size(); i++)
    {
        if(q_r_dot_dot(i)>0.1){
            q_r_dot_dot(i)=0.1;
        }
        else if(q_r_dot_dot(i)<-0.1){
            q_r_dot_dot(i)=-0.1;
        }
    }




}


//----------------------------------------------------------------------------------Y_regressore
void    regressore_Yd()
{

Y_d_temp=regressor(q_r_dot_dot,q_r_dot,dq,q);

int i;


for(int j=0;j<59;j++){
	for(int k=0;k<7;k++){
		i=7*j + k;
		Y_d(k,j)=Y_d_temp(i);
	}
}




}
//---------------------------------------------------------------------------------theta stimata
void    theta_estimate()
{
        theta_cap_dot=-L*Y_d.transpose()*s;

        theta_cap=theta_cap+theta_cap_dot*ts;


                                                }
//-------------------------------------------------------------
void    torque_control(                         
                                                Matrix<double,7,1> q,
                                                Matrix<double,7,1> dq,
                                                Matrix<double, 3, 1> xdes,
                                                Matrix<double, 3, 1> dxdes,
                                                Matrix<double,59,1> theta_cap,
                                                Matrix<double,7,1>  dqr_old                                        
                                                ){
        REGION_CONTROLLER::q=q;
        REGION_CONTROLLER::dq=dq;
        REGION_CONTROLLER::xdes=xdes;
        REGION_CONTROLLER::dxdes=dxdes;
        REGION_CONTROLLER::theta_cap=theta_cap;
        REGION_CONTROLLER::dqr_old=dqr_old;
	std::cout<< "dq_norm: " << dq.norm() << std::endl;
        forward_kinematics();    

        Delta_x=x_e-xdes;

        dominant_region();

        jacobiano();

        x_e_dot= J * dq;


        weight_vector();

        derivate_w_partial();
        transition_matrix_A();

        x_d=x_i + w * ( xdes - x_i );


        delta_x= x_e - x_d;


        //pos_stiffness_tank_large();
	pos_stiffness();

        j_pseudoinverse();
        M= I7 - pinv_J * A * J;


        M_inverse=M.inverse();

	for(int i=0; i<7; i++){
		for(int k=0; k<7; k++){
		
		if(M_inverse(i,k)>1)  M_inverse(i,k)=1;
		if(M_inverse(i,k)<-1)  M_inverse(i,k)=-1;
		
		}

	}
        x_f_dot=w * dxdes - A * dxdes;


        q_r_dot=M_inverse * ( pinv_J * x_f_dot - alpha * pinv_J * k_p * delta_x );




        s=dq-q_r_dot;
	std::cout <<"s_norm: " << s.norm() << std::endl;
	std::cout <<"k: " << k_p << std::endl;
        derivate_dqr();

        regressore_Yd();

        theta_estimate();


        tau_m_a= - M.transpose() * J.transpose() * k_p * delta_x;


        tau_m_b= - K_s * s;


        tau_m_c= Y_d * theta_cap;

	std::cout << "tau_m_c: " << Y_d.maxCoeff() << std::endl;
	std::cout << "theta_cap: " << theta_cap.maxCoeff() << std::endl;


        tau_m=tau_m_a+  tau_m_b + tau_m_c ;

std::cout << "tau_m: " << tau_m.transpose() << std::endl;
	

        }

Matrix<double, 7, 1>    get_qr_dot(){ return q_r_dot; }
Matrix<double, 59, 1>   get_theta(){ return theta_cap; }
Matrix<double, 7, 1>    get_tau_m(){ return tau_m;  }


REGION_CONTROLLER( Matrix<double, 3, 1> x_i,
                        double a_h,
                        double a_r,
                        double a_t,
                        double alpha,
                        double k_max,
                        double ts,
                        Matrix<double, 7, 7> K_s,
                        Matrix<double, 59, 59> L,
                        double k_pinvJ

                        ){
                        REGION_CONTROLLER::x_i=x_i;
                        REGION_CONTROLLER::a_h=a_h;
                        REGION_CONTROLLER::a_r=a_r;
                        REGION_CONTROLLER::a_t=a_t;
                        REGION_CONTROLLER::alpha=alpha;
                        REGION_CONTROLLER::k_max=k_max;
                        REGION_CONTROLLER::ts=ts;
                        REGION_CONTROLLER::K_s=K_s;
                        REGION_CONTROLLER::L=L;
                        REGION_CONTROLLER::k_pinv_J=k_pinvJ;
              };

};

