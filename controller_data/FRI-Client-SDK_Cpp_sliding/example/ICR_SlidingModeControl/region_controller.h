#ifndef _REGION_CONTROLLER_H
#define _REGION_CONTROLLER_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
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
    Matrix<double, 7, 1> q;                 // Joint position vector
    Matrix<double, 7, 1> q_old;             // Joints old position
    Matrix<double, 7, 1> dq;                // Joint velocity vector
    Matrix<double, 6, 1> xdes;              // desired ee position
    Matrix<double, 6, 1> dxdes;             // desired ee velocity
    Matrix<double, 6, 1> x_i;               // time invariant reference
    Matrix<double, 3, 1> phi;                //orientation vector
    Matrix<double, 3, 3> T_phi;              // Mapping matrix angular-eurler velocity
    Matrix<double, 6, 6> T;                  // Mapping matrix analitical-geometric Jacobian  
    double a_h;                             // H-DR radius
    double a_r;                             // R-DR radius
    double a_t;                             // TS-SR radius
    double alpha;                           // position gain into sliding vector
    double k_1;                             // gain position constant
    double ts;                              // sampling time
    Matrix<double, 7, 7> K_s;               // sliding vector gain matrix
    Matrix<double, 6, 1> x_e;               // ee position
    Matrix<double, 6, 1> x_e_dot;           // ee velocity
    Matrix<double, 6, 1> Delta_x;           // position error into operative space
    std::string region;                     // name of active region
    double w;                               // weighted factor
    double dw_part;                         // partial derivate of w
    Matrix<double,6,6> A;                   // transition matrix
    Matrix<double, 6, 1> x_d;               // reference position weighted
    Matrix<double, 6, 1> x_d_dot;           // reference velocity weighted
    Matrix<double, 6, 1> delta_x;           // position weighted error
    double k_p;                             // position gian
    Matrix<double,6,7>J;                    // jacobian (6x7)
    Matrix<double,7,6> pinv_J;              // pseudo-inverse jacobian
    double k_pinv_J;                        // damping factor for Jplus
    Matrix<double, 7, 7> M;                 // Modifier matrix
    Matrix<double, 7, 7> I7=Matrix<double, 7, 7>::Identity();   //eye(7)
    Matrix<double, 6, 6> I6=Matrix<double, 6, 6>::Identity();  // eye(6)
    Matrix<double, 3, 3> O3;  // zeros(3)
    Matrix<double, 3, 3> I3=Matrix<double, 3, 3>::Identity();   //eye(3)
    Matrix<double, 7, 7> M_inverse;         // inverse of modifier matrix
    Matrix<double, 7, 1> q_r_dot;           // virtual joint velocity reference
    Matrix<double, 7, 1> s;                 // sliding vector
    Matrix<double, 7, 1> tau_m;             // motion control torque
    double norm_Delta_x;                    // norm of Delta_x
    Matrix<double,6,1> dw_dx;               // partial derivate of w by x
    int dof;                                // degree of freedom
    Matrix<double, 6, 6> Jtemp;             
 	Matrix<double, 4, 4> Tbe;               // trasformation matrix to ee from base frame
    Matrix<double, 3, 3> Rbe;               // rotation matrix to ee base frame
    Matrix<double, 3, 3> Res;               // rotation matrix to force sensor from ee frame
    Matrix<double, 3, 3> Rbs;               // rotation matrix to force sensor from base frame
    Matrix<double, 6, 6> Rbs_6dof;
   	double db=0.0;                          // DH term for link: base
   	double d0=0.34;                         // DH term for link: 0
   	double d3=0.40;                         // DH term for link: 3
   	double d5=0.40;                         // DH term for link: 5
   	double d7=0.126;                        // DH term for link: 7
    bool first_iter;                        // first iteraction (?)
    double f_p;                             // gain exponential function
    double a;                               // time constant into f_p and k_p
    double a_i;                             // radius of maximun k_p
    double k_max;                           // position gain at a_i radius
    double k_p_temp;                        
    double beta;                            // Retail Region radius
    double sigma;                           // projecting region
    double w_s;                             //saturation treshold
    double gamma;                           //angular difference from s_x and f_h
    Matrix<double, 6, 1> s_x;               // operative space sliding vector   
    Matrix<double, 6, 1> f_h;               // interaction human-robot force
    std::string interact_region;            //interaction region
    double mu_s;                            //saturation term
    double mu_x;                            // transition term
    Matrix < double, 6, 1> c_x;             // interaction term
    Matrix < double, 6, 1> f_h_bs;
    double lambda;                          // operative space interaction drag 
    Matrix < double, 2, 1> s_x_cap_unit;    // operative space sliding vector prejected at beta angular distance
    Matrix < double, 6, 1> C_x;             // interaction controller
    Matrix < double, 2 ,1> f_h_2d;          // mesaured force with only 2 components
    Matrix < double, 2 ,1> s_x_2d;          // operative space sliding vector with only 2 components
    Matrix < double ,2 ,2> A_opt;           // matrix to calculate the projected vector s_x' by linear constraints
    Matrix < double ,2 ,1> B_opt;           // A_opt * x = B_opt
    Matrix < double ,2 ,1> s_x_2d_unit;     // unit vector of s_x projected to beta angle
    Matrix < double ,2 ,1> s_x_2d_opt;      // vector of s_x projected to beta angle from it
    Matrix < double ,6 ,6> K_P;             // position gain matrix
    Matrix < double ,7 ,1> tau_i;           // interaction control torque
    Matrix < double ,7 ,1> tau;             // control torque
    Matrix < double ,6 ,1> s_x_temp;        // temporary vector sx
    Matrix<double, 7, 1> tau_m_a;           // sliding control torque
    Matrix<double, 7, 1> tau_m_b;           // adaptive control torque
    Matrix<double, 7, 1> tau_m_temp;      
    double k_p_old;
    double k_p_new;
    Matrix<double, 7, 1> dq_deg;

/*
    double ml1=3.4525;
    double ml2=3.4821;
    double ml3=4.05623;
    double ml4=3.4822;
    double ml5=2.1633;
    double ml6=2.3466;
    double ml7=3.129;
    double g0=9.81;
*/
    double ml1=0;
    double ml2=0;
    double ml3=0;
    double ml4=0;
    double ml5=0;
    double ml6=0;
    double ml7=0.75;
    double g0=9.81;
    double mm1=0;
    double mm2=mm1;
    double mm3=mm2;
    double mm4=mm3;
    double mm5=mm1;
    double mm6=mm2;
    double mm7=mm3;
    double q1;
    double q2;
    double q3;
    double q4;
    double q5;
    double q6;
    double q7;
    Matrix<double,7,1> G;
public:
/*******************************************************************************
 * function to calculare end-effector position from joint position
 * *****************************************************************************/
void forward_kinematics();


/*******************************************************************************
 * function to calculate which dominant region is active
 * *****************************************************************************/
void    dominant_region();


/*******************************************************************************
 * function to check if a_i radius exceed a_r, if happen, the a_r radius will be increased
 * *****************************************************************************/
void check_gain_parameters();


/*******************************************************************************
 * function to calculare weigthed term w 
 * *****************************************************************************/
void    weight_vector();


/*******************************************************************************
 * function to calculare  dw/dx_i where i is the i-th element of x_e
 * *****************************************************************************/
void    derivate_w_partial();


/*******************************************************************************
 * function to calculare transition matrix A with dw_part
 * *****************************************************************************/
void    transition_matrix_A();


/*******************************************************************************
 * function to calculare gain k_p that will be apply to the position controller
 * 
 * [in] (double) position error norm
 * [out] (double) position gain value
 * 
 * *****************************************************************************/
double   pos_stiffness(double delta_x_norm);


/*******************************************************************************
 * function to calculare 6 dimension jacobian
 * *****************************************************************************/
void    jacobiano();


/*******************************************************************************
 * function to calculate pseudo-inverse of J
 * *****************************************************************************/
void    j_pseudoinverse();


/**********************************************************
 * function to calcute angle distant gamma between -sx and fh
 * * and determiante in which interaction region is active
 * *********************************************************/ 
void interactive_region();


/*******************************************************************************
 * function to calculare saturaction function mu_s inside interaction controller 
 * *****************************************************************************/
void saturation_function();


/*******************************************************************************
 * function to calculare transition function mu_x inside interaction controller
 * *****************************************************************************/
void transition_function();


/*******************************************************************************
 * function to calculare interaction vector c_x from f_h
 * *****************************************************************************/
void interaction_vector_function();


/*******************************************************************************
 * function to calculare interaction controller C_x from c_x, mu_s and mu_x
 * *****************************************************************************/
void interaction_controller_function();


/************************************************************
 * function to calculate optimal vector with angle difference equal to beta from -sx and
 * equal to (gamma - sigma) from fh
 * ************************************************************/
void optimal_vector();


/*******************************************************************************
 * function to change force sensor frame from sensor to robot base 
 * *****************************************************************************/
void sensor2base();


/*******************************************************************************
 * function to check if sx's norm were lower than 1e-3
 * 'cause a very low sx's norm  may bring unexpacted behavior into interaction controller
 * *****************************************************************************/
void checking_sx();


void gravity_vector();
/*******************************************************************************
 * function to calculare the controll torque
 * *****************************************************************************/
void                    torque_control( Matrix<double, 7,1> q, Matrix<double, 6, 1> xdes, Matrix<double, 6, 1> dxdes, Matrix<double, 6, 1> f_h);



/*******************************************************************************
 * some get functiones
 * *****************************************************************************/
Matrix<double, 7, 1>    get_tau_m();
Matrix<double, 6, 1>    get_f_i();
Matrix<double, 7, 1>    get_tau();
Matrix<double, 7, 1>    get_q();
Matrix<double, 7, 1>    get_dq();
Matrix<double, 7, 1>    get_dqr();
double                  get_M();
double                  get_A();
double                  get_kp();
Matrix<double, 7, 1>    get_s();
double                  get_w();
string                  get_region();
double                  get_norm_Delta_x();
Matrix<double,6,1>      get_delta_x();
Matrix<double,6,1>      get_Delta_x();
Matrix<double,6,1>      get_x_e();
double                  get_mus();
double                  get_mux();
Matrix<double,6,1>      get_Cx();
Matrix<double,6,1>      get_cx();
double                  get_gamma();
Matrix<double,2,1>      get_s_unit();
Matrix<double, 7, 1>    get_tau_m_a();
Matrix<double, 7, 1>    get_tau_m_b();
Matrix<double, 6, 1>    get_sx();
Matrix<double, 6, 1>    get_fh();
Matrix<double,6,1>      get_x_e_dot();
Matrix<double, 7, 1>    get_tau_i();

/*******************************************************************************
 * builder and inizilizer 
 * *****************************************************************************/
REGION_CONTROLLER( Matrix<double, 6, 1> x_i,
                        double a_h,
                        double a_r,
                        double a_t,
                        double alpha,
                        double k_1,
                        double ts,
                        Matrix<double, 7, 7> K_s,
                        double k_pinvJ,
                        double a,
			            double beta,
                        double sigma,
                        double w_s,
                        double lambda
                        );

/*******************************************************************************
 * destroyer
 * *****************************************************************************/
 ~REGION_CONTROLLER();
};

#endif 