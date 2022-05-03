
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
using namespace Eigen;
using namespace std;
class REGION_CONTROLLER
{
private:
    Matrix<double, 7, 1> q;                 // Joints position  
    Matrix<double, 7, 1> q_old;             // Joints old position
    Matrix<double, 7, 1> dq;                // Joints velocity
    Matrix<double, 3, 1> xdes;              // position trajectory
    Matrix<double, 3, 1> dxdes;             // velocity trajectory
    Matrix<double, 3, 1> x_i;               // time invariant reference
    double a_h;                             // radius HDR
    double a_r;                             // radius RDR
    double a_t;                             // radius SSR
    double alpha;                           // positive gain
    double k_max;                           // max position gain
    double ts;                              // sample time
    Matrix<double, 7, 7> K_s;               // sliding gain
    Matrix<double, 3, 1> x_e;               // end-effector position
    Matrix<double, 3, 1> x_e_dot;           // end-effector velocity
    Matrix<double, 3, 1> Delta_x;           // position error
    double norm_Delta_x;                    // weighed position error norm
    std::string region;                     // region dominant
    double w;                               // weight term
    double dw_part;                         // weight term derivate
    Matrix<double,3,3> A;                   // transition matrix
    Matrix<double, 3, 1> x_d;               // weighed position reference
    Matrix<double, 3, 1> delta_x;           // weighed position error
    double k_p;                             // position controller gain
    Matrix<double,3,7>J;                    // Jacobian 3 dof
    Matrix<double,7,3> pinv_J;              // jacobian pseudo-inverse 
    double k_pinv_J;                        // jacobian pseudo-inverse damping factor
    Matrix<double, 7, 7> M;                 // Modifier matrix
    Matrix<double, 7, 7> I7=Matrix<double, 7, 7>::Identity(); // identity matrix 7x7 
    Matrix<double, 3, 3> I3=Matrix<double, 3, 3>::Identity(); // identity matrix 3x3
    Matrix<double, 7, 7> M_inverse;         // modifier matrix inverse
    Matrix<double, 3, 1> x_f_dot;           // operative space virtual reference
    Matrix<double, 7, 1> q_r_dot;           // joint virtual reference
    Matrix<double, 7, 1> dqr_old;           // old q_r_dot -- used to derivate q_r_dot --
    Matrix<double, 7, 1> s;                 // sliding vector
    Matrix<double, 7, 1> q_r_dot_dot;       // derivate of q_r_dot
    Matrix<double, 7, 1> tau_m_a;           // position controller torque   
    Matrix<double, 7, 1> tau_m_b;           // sliding controller torque
    Matrix<double, 7, 1> tau_m;             // movement controller torque
    Matrix<double,3,1> dw_dx;               // derivate w respect x_e
    int dof;                                // degree of freedom
    Matrix<double, 3, 3> Jtemp;
	double a;                               // exponential coefficent of gain controller
    Matrix<double, 4, 4> Tbe;               // trasformation matrix base-end effector
    double db=0.0;                          // z axis distance links
    double d0=0.34; 
    double d3=0.40; 
   	double d5=0.40; 
    double d7=0.126;
    bool first_iter;                        // first iteraction (?)
    double f_p;                             // position gain function

public:
//froward kinematics function
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

    /* ----- Print The Data ----- */
    printf("X : %f \n", x_e(0));
    printf("Y : %f \n", x_e(1));
    printf("Z : %f \n", x_e(2));

   
}

// Dominant Region (HDR - RDR - SSR)
void    dominant_region()
{
    norm_Delta_x = Delta_x.norm();


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

// weight term  region based
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

//  derivate dw/dx_i where i is the i-th element of x_e
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

// transition matrix A
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

/* =============================================
Function Name:
Descrption:
Input:
Output:
================================================*/
//position controller gain
void    pos_stiffness()
{

    if (region.compare("H-DR") == 0)
    {
        k_p= 0;
    }
    else if (region.compare("R-DR") == 0)
    {

	    f_p=1 - exp( - a * ( pow ( delta_x.norm() , 2 ) - pow ( a_h ,2 ) ) );
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

// 3 d.o.f Jacobian
void    jacobian()
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
*/





}

// Jacobian pseudo-inverse
void    j_pseudoinverse()
{
                                                    
Jtemp = (J * J.transpose() + I3 * k_pinv_J);
pinv_J = J.transpose() * Jtemp.inverse();

}









// torque calculus
void    torque_control(                         
                                                Matrix<double,7,1> q,
                                                Matrix<double, 3, 1> xdes,
                                                Matrix<double, 3, 1> dxdes,
						Matrix<double, 3, 7> Jext
                                                ){
        REGION_CONTROLLER::q=q;
        REGION_CONTROLLER::xdes=xdes;
        REGION_CONTROLLER::dxdes=dxdes;
	
        forward_kinematics();    

        Delta_x = x_e - xdes;  /* in meter */

        dominant_region();

        jacobian();

        x_e_dot = J * dq;

        weight_vector();

        derivate_w_partial();

        transition_matrix_A();

        x_d = x_i + w * ( xdes - x_i );

        delta_x = x_e - x_d;

	    pos_stiffness();

        j_pseudoinverse();

        M = I7 - pinv_J * A * J;

        M_inverse = M.inverse();
        // saturation M_inverse
	    for(int i=0; i<7; i++){
		    for(int k=0; k<7; k++){
		
		    if(M_inverse(i,k)>1)  M_inverse(i,k)=1;
		    if(M_inverse(i,k)<-1)  M_inverse(i,k)=-1;
    
		    }

	    }

        x_f_dot = w * dxdes - A * dxdes;

        q_r_dot = M_inverse * ( pinv_J * x_f_dot - alpha * pinv_J * k_p * delta_x );

        s = dq - q_r_dot;

        tau_m_a = - M.transpose() * J.transpose() * k_p * delta_x;

        tau_m_b = - K_s * s;

       	tau_m = tau_m_a +  tau_m_b;


        /* ================================ */
/*        Matrix<double,3,7> J3_6;
        for (int i=0; i<6; i++)
        {
            for(int k=0;k<3;k++)
            {
                J3_6(k,i)=J(k,i);
            }
        }
	J3_6(0,6)=0;
	J3_6(1,6)=0;
	J3_6(2,6)=0;
        Matrix<double,7,3> pinv_J3_6;
        Jtemp = (J3_6 * J3_6.transpose() + I3 * k_pinv_J);
        pinv_J3_6 = J3_6.transpose() * Jtemp.inverse();
*/

	Matrix<double,7,3> pinv_J3_6;
        Jtemp = (Jext * Jext.transpose() + I3 * k_pinv_J);
        pinv_J3_6 = Jext.transpose() * Jtemp.inverse();
        delta_x = x_e - xdes;
        x_f_dot = dxdes;
        k_p=1;
        double k_s=0.1;
        alpha=0.5;
        q_r_dot = ( pinv_J3_6 * x_f_dot - alpha * pinv_J3_6 * k_p * delta_x );
//        std::cout << "last pinvJ row: " <<pinv_J3_6(6,0) << pinv_J3_6(6,1) <<  pinv_J3_6(6,2) << " dq7: " << dq(6)<< std::endl;
        s = dq - q_r_dot;

        tau_m_b = - k_s * s;
//	tau_m_b=-k_s*( dq - pinv_J3_6 * x_f_dot) - k_p*pinv_J3_6*delta_x;
        //tau_m_b(6)=0;
        tau_m = tau_m_b;
        //std::cout << "Torque:\n" << tau_m.transpose() << std::endl;
        std::cout << "Delta X:\n" << delta_x.transpose() << std::endl;
        /* ================================= */




        if (!first_iter)
        {
            dq = (q-q_old)/ts;
        }
        first_iter = false;
        q_old = q;
        }


Matrix<double, 7, 1>    get_tau_m(){ return tau_m;  }
double    get_norm_Delta_x(){ return norm_Delta_x;  }
Matrix<double,3,1>    get_delta_x(){ return delta_x;  }



REGION_CONTROLLER(Matrix<double, 3, 1> x_i,
                        double a_h,
                        double a_r,
                        double a_t,
                        double alpha,
                        double k_max,
                        double ts,
                        Matrix<double, 7, 7> K_s,
                        double k_pinvJ,
                        double a)
{
                        REGION_CONTROLLER::x_i=x_i;
                        REGION_CONTROLLER::a_h=a_h;
                        REGION_CONTROLLER::a_r=a_r;
                        REGION_CONTROLLER::a_t=a_t;
                        REGION_CONTROLLER::alpha=alpha;
                        REGION_CONTROLLER::k_max=k_max;
                        REGION_CONTROLLER::a=a;                        
                        REGION_CONTROLLER::ts=ts;
                        REGION_CONTROLLER::K_s=K_s;
                        REGION_CONTROLLER::k_pinv_J=k_pinvJ;  
                        first_iter=true;                
};

};

