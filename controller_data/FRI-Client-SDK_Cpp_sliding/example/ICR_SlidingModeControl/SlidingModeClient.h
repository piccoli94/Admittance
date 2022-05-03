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

#include "friLBRClient.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "socketHelper.h"
#include "region_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include "boost/thread/thread.hpp"


/* Configuration for ATI sensor */
#define OUTPUT_PORT 9090

//using namespace srd Eigen;
using namespace Eigen;
using namespace std;

/* Number of sampling for force filter */
#define N_samp 5

typedef struct ATIdata_struct {
   float fx;
   float fy;
   float fz;
   float tx; 
   float ty; 
   float tz;
} ATIdata_struct;

/**
 * \brief Template client implementation.
 */
class SlidingModeClient : public KUKA::FRI::LBRClient
{
   
public:
      

      
   /**
    * \brief Constructor.
    * 
    * @param jointMask Bit mask that encodes the joint indices to be overlaid by sine waves
    * @param freqHz Sine frequency in Hertz
    * @param torqueAmplitude Sine amplitude in Nm
    */
   //SlidingModeClient(unsigned int jointMask, double freqHz, double torqueAmplitude);
   

   SlidingModeClient(uint8_t u);

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
   virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
   
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
   virtual void command();

   void ATI_data_cb();

   private:

   REGION_CONTROLLER *controller;
   Matrix<double,6,1> x_des;        //Position Reference      
   Matrix<double,6,1> x_des_dot;    //Velocity Reference
   Matrix<double,7,1> q;            //Joint position vector
   Matrix<double,7,1> tau;        //controller torque
   Matrix<double, 7, 7> K_s=Matrix<double, 7, 7>::Identity();        //sliding gain matrix
   double ts;  //sample time
   int count;  //counter
   double k_pinv_J; //presudo inverse damping factor
   double alpha;  // positional gain into sliding vector
   Matrix<double, 6, 1> x_i;        //time invariant reference
   double a_h;                      //HDR radius
   double a_r;                      //RDR radius
   double a_t;                      //TSSR radius
   double k_1;                    //position gain
   double a;                        //positive time constant of gain function
   double w_s;
   double beta;
   double sigma;
   double lambda;
   Matrix< double, 6, 1> f_h; 





   /* Support variables for ATI communication */
   int ati_socket_read;
   int slen;
   int rlen; 
   sockaddr_in si_me, si_other;
   unsigned char buf[24];
   ATIdata_struct ATI_input;
   float time_threshold_count;
   Matrix< double, 6, 1> force_bias;
   Matrix< double, 6, 1> fm;
   int cont;

   std::string z_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/z.txt";
   std::string dz_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/dz.txt";
   std::ifstream file_z;
   std::ifstream file_dz;
   std::vector<double> z_vec;
   std::vector<double> dz_vec;
   std::string y_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/y.txt";
   std::string dy_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/dy.txt";//dye.txt
   std::ifstream file_y;
   std::ifstream file_dy;
   std::vector<double> y_vec;
   std::vector<double> dy_vec;

   std::ofstream file_M;
   std::ofstream file_w;
   std::ofstream file_kp;
   std::ofstream file_s;
   std::ofstream file_q;
   std::ofstream file_tau;
   std::ofstream file_Delta;
   std::ofstream file_xe;
   std::ofstream file_ref;
   std::ofstream file_A;
   std::ofstream file_tau_a;
   std::ofstream file_tau_b;
   std::ofstream file_dxe;
   std::ofstream file_dref;
   std::ofstream file_sx;

   string tau_a_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/tau_a.txt";
   string tau_b_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/tau_b.txt";
   string tau_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/tau.txt";
   string Delta_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/Delta_x.txt";
   string xe_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/x_e.txt";
   string xref_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/x_ref.txt";
   string M_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/M.txt";
   string w_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/w.txt";
   string kp_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/kp.txt";
   string s_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/s.txt";
   string q_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/q.txt";
   string A_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/A.txt";
   string dxe_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/dx_e.txt";
   string dxref_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/dx_ref.txt";
   string sx_name="/home/icaros-pc/Desktop/Piccoli/controller_data/data_02_14/sx.txt";

   int _jointMask;         //!< bit mask encoding of overlay joints
   double _freqHz;         //!< sine frequency (Hertz)
   double _torqueAmpl;     //!< sine amplitude (Nm)
   double _phi;            //!< phase of sine wave
   double _stepWidth;      //!< stepwidth for sine 
   double _torques[KUKA::FRI::LBRState::NUMBER_OF_JOINTS]; //!< commanded superposed torques
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
