/* ======================================================================================
   ********************************* REGION CONTROLLER ******************************

                               All rights are reserved by ICAROS
                               
Description: REGION CONTROLLER for rehablitation of upper/lower limb using a KUKA MED 7

File Name: region_controller


Author: Piccoli Danilo
Email : piccoli94@gmail.com
Date  : Feb 2022


Version:
--------------
V1_2022_02_07:    Version #1 - Last one

==========================================================================================*/




#include "region_controller.h"




/*******************************************************************************
 * function to calculare end-effector position from joint position
 * *****************************************************************************/
void REGION_CONTROLLER::forward_kinematics()
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


    for(int i=0; i<3; i++)
	{
		for(int k=0; k<3; k++)
		{
			Rbe(i,k) = Tbe(i,k);
		}
	}
    phi = Rbe.eulerAngles(2,1,0);
    x_e << Tbe(0,3), Tbe(1,3), Tbe(2,3), phi ; 

    T_phi << 1,             0,                   sin(phi(1)),
             0,   cos(phi(0)),      -cos(phi(1))*sin(phi(0)),
             0,   sin(phi(0)),       cos(phi(1))*cos(phi(0));
    
    T <<    I3,           O3,
            O3,  T.inverse();


}

/*******************************************************************************
 * function to calculate which dominant region is active
 * *****************************************************************************/
void    REGION_CONTROLLER::dominant_region()
{
    norm_Delta_x = Delta_x.norm();
 //std::cout << "norm_Delta_x: " << norm_Delta_x << std::endl;

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

/*******************************************************************************
 * function to check if a_i radius exceed a_r, if happen, the a_r radius will be increased
 * *****************************************************************************/
void REGION_CONTROLLER::check_gain_parameters()
{
    a_i = pow( ( a * ( a * a_h*a_h + 1.0986 ) ), 0.5 )/a;

    if(a_i >= a_r)
    {
        cout << "a_i: " << a_i << " a_r: " << a_r << " ERROR, ROBOTIC REGION RADIUS MUST BE GREATER THEN a_i"<< endl;
        a_r += 0.1 + ( a_i - a_r );
        a_h += 0.1 + ( a_i - a_r );
        cout << "a_r AND a_h WILL BE INCREASED OF " << 0.1 + ( a_i - a_r ) << " METERS" << endl;
    }

    k_max = REGION_CONTROLLER::pos_stiffness(a_i);

    cout << "norm of maximum gain: "<< a_i << " maximum gain: " << k_max << endl;

} 
    

/*******************************************************************************
 * function to calculare weigthed term w 
 * *****************************************************************************/
void    REGION_CONTROLLER::weight_vector()
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

/*******************************************************************************
 * function to calculare  dw/dx_i where i is the i-th element of x_e
 * *****************************************************************************/
void    REGION_CONTROLLER::derivate_w_partial()
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

/*******************************************************************************
 * function to calculare transition matrix A with dw_part
 * *****************************************************************************/
void    REGION_CONTROLLER::transition_matrix_A()
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

/*******************************************************************************
 * function to calculare gain k_p that will be apply to the position controller
 * 
 * [in] (double) position error norm
 * [out] (double) position gain value
 * 
 * *****************************************************************************/
double  REGION_CONTROLLER::pos_stiffness(double delta_x_norm)
{

    if (region.compare("H-DR") == 0)
    {
        k_p_temp= 0;
    }
    else if (region.compare("R-DR") == 0)
    {

	 f_p=1 - exp( - a * ( pow ( delta_x_norm , 2 ) - pow ( a_h ,2 ) ) );
        k_p_temp= k_1 * pow( std::max( f_p , 0.0 ), 2 )  *  exp( - a * ( pow ( delta_x_norm , 2 ) - pow ( a_h ,2 ) ) );
    }
    else if (region.compare("TS-SR") == 0)
    {
        k_p_temp= 0;
    }
    else if (region.compare("SS-SR") == 0)
    {
        k_p_temp=0;
    }
        return k_p_temp;
    }

/*******************************************************************************
 * function to calculare 6 dimension analitical jacobian
 * *****************************************************************************/
void    REGION_CONTROLLER::jacobiano()
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
	J(3,0)=0.0;
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

    J = T * J; //mapping geometrical jacobian into analitical


}

/*******************************************************************************
 * function to calculate pseudo-inverse of J
 * *****************************************************************************/
void    REGION_CONTROLLER::j_pseudoinverse()
{
                                                    
Jtemp = (J * J.transpose() + I6 * k_pinv_J);
pinv_J = J.transpose() * Jtemp.inverse();

}

/**********************************************************
 * function to calcute angle distant gamma between -sx and fh
 * and determiante in which interaction region is active
 * *********************************************************/ 
void REGION_CONTROLLER::interactive_region()
{
    
    s_x_2d(0) = s_x (1); // suppose to have a force control only in y and z
    s_x_2d(1) = s_x (2);
    f_h_2d(0) = f_h_bs (1);
    f_h_2d(1) = f_h_bs (2);

    double temp = -s_x_2d.transpose() * f_h_2d;

    if ( f_h_2d.norm() > 0 ) gamma = acos( ( temp ) / ( s_x_2d.norm() * f_h_2d.norm() ) );
        else  gamma =0;

    if(gamma <= beta) 
    {
        interact_region = "RR" ;
        
    }
    else if( gamma > beta && gamma <= ( sigma + beta ) )
    {
        interact_region = "PR" ;
    }
    else if( gamma > ( sigma + beta ) )
    {
        interact_region = "CR" ;
    }
}
/*******************************************************************************
 * function to calculare saturaction function mu_s inside interaction controller 
 * *****************************************************************************/
void REGION_CONTROLLER::saturation_function()
{

    if( s_x.norm() >= w_s )
    {
        mu_s = 1;
    }
    else if( s_x.norm() < w_s )
    {
        mu_s = pow( sin( s_x.norm() * M_PI / ( 2 * w_s ) ), 2 );
    }

}


/*******************************************************************************
 * function to calculare transition function mu_x inside interaction controller
 * *****************************************************************************/
void REGION_CONTROLLER::transition_function()
{
    if( region.compare("H-DR") == 0 || region.compare("TS-SR") == 0 || region.compare("SS-SR") == 0 )
    {

        mu_x = 1;

    }
    else if( region.compare("R-DR") == 0 )
    {

        mu_x = sin( ( ( a_r - norm_Delta_x ) * M_PI ) / ( 2 * ( a_r - a_h ) ) );

    }
}
/*******************************************************************************
 * function to calculare interaction vector c_x from f_h
 * *****************************************************************************/
void REGION_CONTROLLER::interaction_vector_function()
{
    if ( interact_region.compare("RR") == 0 )
    {
        c_x <<  f_h(0) * lambda, 
                f_h_2d(0) * lambda, 
                f_h_2d(1) * lambda,
                0,
                0,
                0;
    }
    else if ( interact_region.compare("PR") == 0 )
    {

        REGION_CONTROLLER::optimal_vector();

        c_x  << 0,
                0,
                0,
                0,
                0,
                0;

    }
    else if ( interact_region.compare("CR") == 0 )
    {
        c_x <<  0, 
                0, 
                0,
                0,
                0,
                0;
    }

}
/*******************************************************************************
 * function to calculare interaction controller C_x from c_x, mu_s and mu_x
 * *****************************************************************************/
void REGION_CONTROLLER::interaction_controller_function()
{
    if( region.compare("H-DR") == 0 )
    {
        C_x = mu_s * c_x;
    }
    else if( region.compare("R-DR") == 0 )
    {
        C_x = mu_s * mu_x * c_x;
    }
    else
    {
        C_x << 0, 0, 0, 0, 0, 0;
    }

    C_x = C_x - f_h_bs;//
}

/************************************************************
 * function to calculate optimal vector with angle difference equal to beta from -sx and
 * equal to gamma - sigma from fh
 * ************************************************************/
void REGION_CONTROLLER::optimal_vector()
{
    A_opt << -s_x_2d(0) , -s_x_2d(1),
             f_h_2d(0) , f_h_2d(1);
    
    B_opt << s_x_2d.norm() * s_x_2d.norm() * cos( beta ),
             s_x_2d.norm() * f_h_2d.norm() * cos( gamma - beta );

    if( A_opt.determinant() != 0 ) s_x_2d_opt = A_opt.inverse() * B_opt;
    else 
    {
        s_x_2d_opt << 0, 0;
        cout << "A NULL" << endl;
    }
    if( s_x_2d_opt.norm() != 0 )    s_x_2d_unit = s_x_2d_opt / s_x_2d_opt.norm();
    else s_x_2d_unit = s_x_2d_opt;
}
/*******************************************************************************
 * function to change force sensor frame from sensor to robot base 
 * *****************************************************************************/
void REGION_CONTROLLER::sensor2base()
{
    Res <<  1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    Rbs = Rbe * Res;
    Rbs_6dof << Rbs, O3, Rbs, O3;
    f_h_bs = Rbs_6dof * f_h;
            
}

/*******************************************************************************
 * function to check if sx's norm were lower than 1e-3
 * 'cause a very low sx's norm  may bring unexpacted behavior into interaction controller
 * *****************************************************************************/
void REGION_CONTROLLER::checking_sx()
{
/*    if(s_x_temp.norm() > 0.001)
    {
        s_x = s_x_temp;
    }
*/
}


void REGION_CONTROLLER::gravity_vector()
{
q1=q(0);
q2=q(1);
q3=q(2);
q4=q(3);
q5=q(4);
q6=q(5);
q7=q(6);

G <<                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0,
- g0*ml7*(d3*sin(q2) + d5*cos(q4)*sin(q2) - d5*cos(q2)*cos(q3)*sin(q4) + (9*d7*cos(q4)*cos(q6)*sin(q2))/8 - (9*d7*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 - (9*d7*cos(q2)*sin(q3)*sin(q5)*sin(q6))/8 + (9*d7*cos(q5)*sin(q2)*sin(q4)*sin(q6))/8 + (9*d7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/8) - g0*ml6*(d3*sin(q2) + d5*cos(q4)*sin(q2) - d5*cos(q2)*cos(q3)*sin(q4)) - g0*ml4*(d3*sin(q2) + (d5*cos(q4)*sin(q2))/4 - (d5*cos(q2)*cos(q3)*sin(q4))/4) - g0*ml5*(d3*sin(q2) + (5*d5*cos(q4)*sin(q2))/4 - (5*d5*cos(q2)*cos(q3)*sin(q4))/4) - g0*mm6*(d3*sin(q2) + d5*cos(q4)*sin(q2) - d5*cos(q2)*cos(q3)*sin(q4)) - g0*mm7*(d3*sin(q2) + d5*cos(q4)*sin(q2) - d5*cos(q2)*cos(q3)*sin(q4)) - g0*mm5*(d3*sin(q2) + (d5*cos(q4)*sin(q2))/2 - (d5*cos(q2)*cos(q3)*sin(q4))/2) - (d3*g0*ml2*sin(q2))/4 - (5*d3*g0*ml3*sin(q2))/4 - (d3*g0*mm3*sin(q2))/2 - d3*g0*mm4*sin(q2),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  - (g0*ml7*sin(q2)*(8*d5*sin(q3)*sin(q4) + 9*d7*cos(q6)*sin(q3)*sin(q4) - 9*d7*cos(q3)*sin(q5)*sin(q6) - 9*d7*cos(q4)*cos(q5)*sin(q3)*sin(q6)))/8 - (d5*g0*ml4*sin(q2)*sin(q3)*sin(q4))/4 - (5*d5*g0*ml5*sin(q2)*sin(q3)*sin(q4))/4 - d5*g0*ml6*sin(q2)*sin(q3)*sin(q4) - (d5*g0*mm5*sin(q2)*sin(q3)*sin(q4))/2 - d5*g0*mm6*sin(q2)*sin(q3)*sin(q4) - d5*g0*mm7*sin(q2)*sin(q3)*sin(q4),
                                                                                                                                                                                                                                                             g0*ml7*(d5*cos(q3)*cos(q4)*sin(q2) - d5*cos(q2)*sin(q4) - (9*d7*cos(q2)*cos(q6)*sin(q4))/8 + (9*d7*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (9*d7*cos(q2)*cos(q4)*cos(q5)*sin(q6))/8 + (9*d7*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/8) - g0*ml6*(d5*cos(q2)*sin(q4) - d5*cos(q3)*cos(q4)*sin(q2)) - g0*mm6*(d5*cos(q2)*sin(q4) - d5*cos(q3)*cos(q4)*sin(q2)) - g0*mm7*(d5*cos(q2)*sin(q4) - d5*cos(q3)*cos(q4)*sin(q2)) - (d5*g0*ml4*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)))/4 - (5*d5*g0*ml5*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)))/4 - (d5*g0*mm5*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)))/2,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           (9*d7*g0*ml7*sin(q6)*(cos(q5)*sin(q2)*sin(q3) - cos(q2)*sin(q4)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)))/8,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -(9*d7*g0*ml7*(cos(q2)*cos(q4)*sin(q6) - cos(q2)*cos(q5)*cos(q6)*sin(q4) + cos(q3)*sin(q2)*sin(q4)*sin(q6) - cos(q6)*sin(q2)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)))/8,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
}


/*******************************************************************************
 * function to calculare the controll torque
 * *****************************************************************************/
//-------------------------------------------------------------
void    REGION_CONTROLLER::torque_control(                         
                                                Matrix<double, 7,1> q,
                                                Matrix<double, 6, 1> xdes,
                                                Matrix<double, 6, 1> dxdes,
                                                Matrix<double, 6, 1> f_h
                                                ){
        REGION_CONTROLLER::q = q;
        REGION_CONTROLLER::xdes = xdes;
        REGION_CONTROLLER::dxdes = dxdes;
        REGION_CONTROLLER::f_h = f_h;
        

        REGION_CONTROLLER::forward_kinematics(); 

        REGION_CONTROLLER::sensor2base();

        Delta_x = x_e - xdes;

        REGION_CONTROLLER::dominant_region();
        
        REGION_CONTROLLER::jacobiano();

        x_e_dot = J * dq;

        REGION_CONTROLLER::weight_vector();

        REGION_CONTROLLER::derivate_w_partial();

        REGION_CONTROLLER::transition_matrix_A();

        x_d = x_i + w * ( xdes - x_i );

        delta_x= x_e - x_d;

        k_p = REGION_CONTROLLER::pos_stiffness( Delta_x.norm() );
	    
//        k_p_new 

//        k_p = 0.75 * k_p_old + 0.25 * k_p_new;

//        k_p_old = k_p;

        //double gain=600;
        K_P << k_p,   0,   0,   0,  0,  0,  
                 0, k_p,   0,   0,  0,  0, 
                 0,   0, k_p,   0,  0,  0, 
                 0,   0,   0,  10,  0,  0, 
                 0,   0,   0,   0, 10,  0, 
                 0,   0,   0,   0,  0, 10;

        REGION_CONTROLLER::j_pseudoinverse();

        M = I7 - pinv_J * A * J;

        M_inverse = M.inverse();

	    for(int i=0; i<7; i++){
		    for(int k=0; k<7; k++){
		
		    if( M_inverse(i,k) > 1 )  M_inverse(i,k) = 1;
		    if( M_inverse(i,k) < -1 )  M_inverse(i,k) = -1;
		
		    }

    	}

        x_d_dot = A*x_e_dot + w * dxdes - A * dxdes;

        q_r_dot = pinv_J * x_d_dot - alpha * pinv_J * K_P * delta_x ;

        s =  M_inverse *( dq - q_r_dot ) ; 




        tau_m_a =  - M.transpose() * J.transpose() * K_P * delta_x ; 

        tau_m_b = - K_s * s;

        tau_m =  tau_m_b + tau_m_a ; // 


 /*       if(region.compare("H-DR") == 0 )
        {
            s_x << -x_d_dot(0), -x_d_dot(1) , -x_d_dot(2);  
        }
        else
        {
            s_x = J*s;            
        }
 */      

s_x = J*s;  

 //       REGION_CONTROLLER::checking_sx();

        REGION_CONTROLLER::interactive_region();

        REGION_CONTROLLER::saturation_function();

        REGION_CONTROLLER::transition_function();

        REGION_CONTROLLER::interaction_vector_function();

        REGION_CONTROLLER::interaction_controller_function();

        tau_i = J.transpose() *( C_x );


        REGION_CONTROLLER::gravity_vector();
        

        tau=tau_i  +  tau_m + G  ;// 



        dq = (q-q_old)/ts;         

        q_old = q;
}

/*******************************************************************************
 * some get functiones
 * *****************************************************************************/
Matrix<double, 7, 1>    REGION_CONTROLLER::get_tau_m(){ return tau_m;  }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_tau_i(){ return tau_i;  }
Matrix<double, 6, 1>    REGION_CONTROLLER::get_f_i(){ return pinv_J.transpose()*tau_i;  }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_tau(){ return tau;  }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_q(){ return  q; }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_dq(){ return dq;  }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_dqr(){ return q_r_dot;  }
double                  REGION_CONTROLLER::get_M(){ return M_inverse(1,1);  }
double                  REGION_CONTROLLER::get_A(){ return A(1,1);  }
double                  REGION_CONTROLLER::get_kp(){ return k_p;  }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_s(){ return s;  }
double                  REGION_CONTROLLER::get_w(){ return w;  }
string                  REGION_CONTROLLER::get_region(){ return region; }
double                  REGION_CONTROLLER::get_norm_Delta_x(){ return norm_Delta_x;  }
Matrix<double,6,1>      REGION_CONTROLLER::get_delta_x(){ return delta_x;  }
Matrix<double,6,1>      REGION_CONTROLLER::get_Delta_x(){ return Delta_x;  }
Matrix<double,6,1>      REGION_CONTROLLER::get_x_e(){ return x_e;  }
double                  REGION_CONTROLLER::get_mus(){ return mu_s;  }
double                  REGION_CONTROLLER::get_mux(){ return mu_x;  }
Matrix<double,6,1>      REGION_CONTROLLER::get_Cx(){ return C_x;  }
Matrix<double,6,1>      REGION_CONTROLLER::get_cx(){ return c_x;  }
double                  REGION_CONTROLLER::get_gamma(){ return gamma;  }
Matrix<double,2,1>      REGION_CONTROLLER::get_s_unit(){ return s_x_2d_unit;  }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_tau_m_a(){ return tau_m_a;  }
Matrix<double, 7, 1>    REGION_CONTROLLER::get_tau_m_b(){ return tau_m_b;  }
Matrix<double, 6, 1>    REGION_CONTROLLER::get_fh(){ return f_h_bs;  }
Matrix<double, 6, 1>    REGION_CONTROLLER::get_sx(){ return s_x;  }
Matrix<double,6,1>      REGION_CONTROLLER::get_x_e_dot(){ return x_e_dot;  }


/*******************************************************************************
 * builder and inizilizer 
 * *****************************************************************************/
REGION_CONTROLLER::REGION_CONTROLLER( Matrix<double, 6, 1> x_i,
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
                        ){
                            REGION_CONTROLLER::x_i=x_i;
                            REGION_CONTROLLER::a_h=a_h;
                            REGION_CONTROLLER::a_r=a_r;
                            REGION_CONTROLLER::a_t=a_t;
                            REGION_CONTROLLER::alpha=alpha;
                            REGION_CONTROLLER::k_1=k_1;
                            REGION_CONTROLLER::ts=ts;
                            REGION_CONTROLLER::K_s=K_s;
                            REGION_CONTROLLER::k_pinv_J=k_pinvJ;  
			                REGION_CONTROLLER::a=a;
                            REGION_CONTROLLER::beta=beta;
                            REGION_CONTROLLER::sigma=sigma;
                            REGION_CONTROLLER::w_s=w_s;	
                            REGION_CONTROLLER::lambda=lambda;	
                            //check_gain_parameters();
			                dq << 0,0,0,0,0,0,0;
			                q_old << 0,0,0,-1.57,0,1.57,0;
                            

                        }

