/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.Connectivity FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.Connectivity FastRobotInterface� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2019 
KUKA Roboter GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.15}
*/
#include <cstdio>
#include <cstdio>
#include <string.h>
#include "SlidingModeClient.h"
// Visual studio needs extra define to use math constants
#define _USE_MATH_DEFINES
#include <math.h>


//using namespace KUKA::FRI;

//******************************************************************************
SlidingModeClient::SlidingModeClient(int jointMask/*unsigned , 
      double freqHz, double torqueAmplitude*/) 
   /*
   :_jointMask(jointMask)
   , _freqHz(freqHz)
   , _torqueAmpl(torqueAmplitude)
   , _phi(0.0)
   , _stepWidth(0.0)
   */
{
   /*
   printf("LBRTorqueSineOverlayClient initialized:\n"
         "\tjoint mask: 0x%x\n"
         "\tfrequency (Hz): %f\n"
         "\tamplitude (Nm): %f\n",
         jointMask, freqHz, torqueAmplitude);
   */
   for(int i = 0; i< 7; i++){ _torques[i] = 0.0;}
}

//******************************************************************************
SlidingModeClient::~SlidingModeClient()
{
}
      
//******************************************************************************
void SlidingModeClient::onStateChange(/*ESessionState oldState, ESessionState newState*/)
{
   /*
   LBRClient::onStateChange(oldState, newState);
   // react on state change events
   switch (newState)
   {
      case MONITORING_WAIT:
      {
   */
         for(int i = 0; i< 7; i++)
         { 
            _torques[i] = 0.0;
         }
         _phi        = 0.0;
         _stepWidth  = 0.01;
	      x_i << 0.4,0.6,0.8;
         a_h   =  0.001;
         a_r   =  1;
         a_t   =  1.5;
         alpha =  0.5;
         k_max =  100;
         a     =  1;
         coef_Ks  = 0.5;
         coef_L   = 0.0001;
         K_s   =  Matrix<double,7,7>::Identity()*coef_Ks;
         L     =  Matrix<double,59,59>::Identity()*coef_L;
         k_pinvJ  =  0.001;
         //ts =  robotState().getSampleTime();
         ts=0.01;
         //ts=0.01;
         //controller = new REGION_CONTROLLER(x_i,a_h,a_r,a_t,alpha,k_max,ts,K_s,k_pinvJ,a);
         
         count=0;

         file_theta_read.open(theta_name);
         file_y.open(name_y);
         file_dy.open(name_dy);
	 file_tau.open(tau_name);
         double y_temp=0;
         std::string line;

   	while (std::getline(file_y, line)) {
        	y_temp=atof(line.c_str());
        	y_vec.push_back(y_temp);
    	}
      while (std::getline(file_dy, line)) {
        	y_temp=atof(line.c_str());
        	dy_vec.push_back(y_temp);
    	}
      while (std::getline(file_theta_read, line)) {
        	y_temp=atof(line.c_str());
        	theta_vec.push_back(y_temp);
    	}
      for(int i=0;i<59;i++)
      {
         theta_cap(i)=theta_vec[i];
      }

controller = new REGION_CONTROLLER(ts,L,k_pinvJ,theta_cap);

file_y.close();
file_dy.close();
file_theta_read.close();
printf("y: %f, dy: %f , theta: %f \n", y_vec[1999],dy_vec[1999],theta_cap(39) );

q_start<< 0,0,0-1.57,0,1.57,0;
q_final<< 0.70, 0.88, 0.567, -0.5, 0.35, 1.45, 0;
q_temp << 0,0,0,0,0,0,0;
      /*
         break;
      }       
      case MONITORING_READY:
      {
         break;
      }
      case COMMANDING_WAIT:
      {
         break;
      }   
      case COMMANDING_ACTIVE:
      {
         break;
      }   
      default:
      {
         break;
      }
   }
   */
}

//******************************************************************************
void SlidingModeClient::monitor()
{
   //LBRClient::monitor();
   
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/
   
}

//******************************************************************************
void SlidingModeClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   //LBRClient::waitForCommand();
   
    /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/
   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is 
   // only necessary, that some torque values are sent. The LBR does not take the 
   // specific value into account.
   /*
   if (robotState().getClientCommandMode() == TORQUE)
   {
      robotCommand().setTorque(_torques);
   }
   */
}

//******************************************************************************
Eigen::Matrix<double,7,1> SlidingModeClient::command(Eigen::Matrix<double,7,1> q)
{

   // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    //LBRClient::command();
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/   
    //double Toruqe_Joint[KUKA::FRI::LBRState::NUMBER_OF_JOINTS]; 
    //double Position_Joint[KUKA::FRI::LBRState::NUMBER_OF_JOINTS]; 
    //double Position_Joint[7]; 

   // Check for correct ClientCommandMode.
    /*
    if (robotState().getClientCommandMode() == TORQUE)
    { 
      // calculate  offset
      memcpy(Toruqe_Joint, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      memcpy(Position_Joint, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

      for(int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
      { 
         q(i) = Position_Joint[i];
      }
   */

      //x_des       << 0.4, _phi, 0.614;
      //x_des_dot   << 0, _stepWidth/ts, 0; 

      //x_des << 0.4, 0.1, 0.614;
      //x_des_dot << 0, 0, 0; 
    /* if(_phi==0.0)
     {
         x_des_dot   << 0, 0, 0; 
     }
      _phi += _stepWidth;
       if (_phi >= 0.6) 
       {
           _phi= 0.6; 
           x_des_dot   << 0, 0, 0;
       }
       */
	q_des=q_start + _phi*(q_final-q_start)/(10000*_stepWidth);
if(_phi==0.0)
{
	q_des_dot << 0,0,0,0,0,0,0;
}


     _phi+=_stepWidth;
     q_des_dot=(q_des-q_temp)/ts;


      if(_phi>10000*_stepWidth) _phi=10000*_stepWidth;
	std::cout << "q_des: " << q_des.transpose() << std::endl;
	std::cout << "dq_des: " << q_des_dot.transpose() << std::endl;
	q_des << 0, 0, 0, 0, 0, 0 , 0;
	q_des_dot << 0, 0, 0, 0, 0, 0, 0;
      /* ----- Controller ----- */
	controller->torque_control(q,q_des,q_des_dot);
      tau_m = controller->get_tau_m();
	file_tau << tau_m.transpose();
      file_theta_write.open(theta_name_write);

      file_theta_write << controller->get_theta_cap() << std::endl;
      file_theta_write.close();

	q_temp=q_des;

      for(int i = 0; i< 7; i++)
      { 
         _torques[i] = tau_m(i);
      }
       // Set superposed joint torques.
       //robotCommand().setTorque(_torques);
       
    //}
    return tau_m;
}
// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
