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
#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

//#include "friLBRClient.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "region_controller.cpp"
//#include "region_controller_without_reg.cpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>


/**
 * \brief Template client implementation.
 */
class SlidingModeClient //: public KUKA::FRI::LBRClient
{
   
public:
      

      
   /**
    * \brief Constructor.
    * 
    * @param jointMask Bit mask that encodes the joint indices to be overlaid by sine waves
    * @param freqHz Sine frequency in Hertz
    * @param torqueAmplitude Sine amplitude in Nm
    */
   SlidingModeClient(int jointMask/*unsigned , double freqHz, double torqueAmplitude*/);
   
   /** 
    * \brief Destructor.
    */
   ~SlidingModeClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   //virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
   virtual void onStateChange(/*KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState*/);

   /**
    * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
    * 
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void monitor();
   
   /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    * 
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void waitForCommand();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    * 
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual Eigen::Matrix<double,7,1> command(Eigen::Matrix<double,7,1> q,Eigen::Matrix<double,7,1> dq);

   private:
   REGION_CONTROLLER *controller;
   Matrix<double,3,1> x_des;        //Position Reference
   Matrix<double,3,1> x_des_dot;    //Velocity Reference
   Matrix<double, 3, 1> x_i;        //time invariant reference
   Matrix<double,7,1> tau_m;        //controller torque
   double a_h;                      //HDR radius
   double a_r;                      //RDR radius
   double a_t;                      //TSSR radius
   double alpha;                    //positive gain
   double k_max;                    //position gain
   double coef_Ks;                  //sliding gain
   double a;                        //positive time constant 
   Matrix<double, 7, 7> K_s;        //sliding gain matrix
   Matrix<double, 59, 59> L;
   double coef_L;
   double k_pinvJ;                  // pseudo-inverse jacobian's damping factor
   Matrix<double, 59, 1> theta_cap;        
   double ts;
   double norm_error;
	std::ofstream file_delta;
   std::ofstream file_tau;
	std::ofstream file_theta_write;
   std::ifstream file_y;
   std::ifstream file_dy;
	std::vector<double> y_vec;
	std::vector<double> dy_vec;
   std::ifstream file_theta_read;
	std::vector<double> theta_vec;
	double time_max;
	double time_temp;
	int count_time;
   string theta_name="/home/danilo/catkin_ws/src/lab_test/src/theta_cap.txt";
   string theta_name_write="/home/danilo/catkin_ws/src/lab_test/src/theta_cap_write.txt";

   string tau_name="/home/danilo/catkin_ws/src/lab_test/src/Recording/tau.txt";
string Delta_name="/home/danilo/catkin_ws/src/lab_test/src/Recording/Delta_x.txt";
   string name_y="/home/danilo/catkin_ws/src/lab_test/src/y.txt";
   string name_dy="/home/danilo/catkin_ws/src/lab_test/src/dy.txt";
   int count;
   int _jointMask;         //!< bit mask encoding of overlay joints
   double _freqHz;         //!< sine frequency (Hertz)
   double _torqueAmpl;     //!< sine amplitude (Nm)
   double _phi;            //!< phase of sine wave
   double _stepWidth;      //!< stepwidth for sine 
   double _torques[7]; //!< commanded superposed torques

		string tau_name_a="/home/danilo/catkin_ws/src/lab_test/src/Recording/tau_a.txt";
		string tau_name_b="/home/danilo/catkin_ws/src/lab_test/src/Recording/tau_b.txt";
		string tau_name_c="/home/danilo/catkin_ws/src/lab_test/src/Recording/tau_c.txt";
		std::ofstream file_tau_a;
		std::ofstream file_tau_b;
		std::ofstream file_tau_c;
		string q_name="/home/danilo/catkin_ws/src/lab_test/src/Recording/q.txt";
		string dq_name="/home/danilo/catkin_ws/src/lab_test/src/Recording/dq.txt";
		string dqr_name="/home/danilo/catkin_ws/src/lab_test/src/Recording/q_r_dot.txt";
		string ddqr_name="/home/danilo/catkin_ws/src/lab_test/src/Recording/q_r_dot_dot.txt";
		std::ofstream file_q;
		std::ofstream file_dq;
		std::ofstream file_dqr;
		std::ofstream file_ddqr;
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
