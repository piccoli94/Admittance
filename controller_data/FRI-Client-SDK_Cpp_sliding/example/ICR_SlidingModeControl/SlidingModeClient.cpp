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
#include "friLBRState.h"



using namespace KUKA::FRI;

void SlidingModeClient::ATI_data_cb()
{
	Matrix<double, N_samp ,1> f_temp_x;
   Matrix<double, N_samp ,1> f_temp_y;
	Matrix<double, N_samp ,1> f_temp_z;
	Matrix<double, N_samp ,1> t_temp_x;
   Matrix<double, N_samp ,1> t_temp_y;
	Matrix<double, N_samp ,1> t_temp_z;

	uint8_t count = 0;
	float sum_f_x = 0;
	float sum_f_y = 0;
	float sum_f_z = 0;
	float sum_t_x = 0;
	float sum_t_y = 0;
	float sum_t_z = 0;

	f_temp_x.setZero();
	f_temp_y.setZero();
	f_temp_z.setZero();
	t_temp_x.setZero();
	t_temp_y.setZero();
	t_temp_z.setZero();

	while(1) 
	{
		// Read ATI data 
		slen = sizeof( si_other );
		rlen = recvfrom( ati_socket_read, buf, sizeof(ATIdata_struct), 0, (struct sockaddr*)&si_other, (socklen_t*)&slen);

		if(rlen !=-1) 
		{
			ATI_input = *(( ATIdata_struct *) (buf));
			if( ( fabs(ATI_input.fz) + fabs(ATI_input.fx) + fabs(ATI_input.fy) ) > 1 )
			{
				fm  <<ATI_input.fx, ATI_input.fy, ATI_input.fz, ATI_input.tx, ATI_input.ty, ATI_input.tz;
            fm -= force_bias;
            
			}
			else
			{
				fm <<0, 0, 0, 0, 0, 0;
			}
		}
		else 
		{
			cout << "ATI no data available" << endl;
			fm  <<0, 0, 0, 0, 0, 0;
		}
		
		// moving average filter
		f_temp_x[count] = fm[0];
      f_temp_y[count] = fm[1];
      f_temp_z[count] = fm[2];
      t_temp_x[count] = fm[3];
      t_temp_y[count] = fm[4];
      t_temp_z[count] = fm[5];
		count++;

		if(count == N_samp)
		{
			count = 0;		
		}

		for(uint16_t i=0; i<N_samp; i++)
		{ 
			sum_f_x += f_temp_x[i];
         sum_f_y += f_temp_y[i];
         sum_f_z += f_temp_z[i];
         sum_t_x += t_temp_x[i];
         sum_t_y += t_temp_y[i];
         sum_t_z += t_temp_z[i];
		}	
		fm[0] = sum_f_x / N_samp;
      fm[1] = sum_f_y / N_samp;
      fm[2] = sum_f_z / N_samp;
      fm[3] = sum_t_x / N_samp;
      fm[4] = sum_t_y / N_samp;
      fm[5] = sum_t_z / N_samp;
		sum_f_x = 0;
		sum_f_y = 0;
		sum_f_z = 0;
		sum_t_x = 0;
		sum_t_y = 0;
		sum_t_z = 0;


	}
}


//******************************************************************************
SlidingModeClient::SlidingModeClient(uint8_t u) 
{
    printf("SlidingModeClient initialized: %d\n", u);
    bool socketIsValid = listener_socket(OUTPUT_PORT, &ati_socket_read);

	if(socketIsValid == false)
	{
		exit(-1);	
	}
	else
	{
		cout << "Communication socket with ATI sensor opened." << endl;

	}

	boost::thread ATI_data_t(boost::bind(&SlidingModeClient::ATI_data_cb, this));

	system("rosservice call /ft_sensor/bias_cmd \"cmd: 'bias'\"");
   for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
}

//******************************************************************************
/*
SlidingModeClient::SlidingModeClient(unsigned int jointMask, 
      double freqHz, double torqueAmplitude) 
   :_jointMask(jointMask)
   , _freqHz(freqHz)
   , _torqueAmpl(torqueAmplitude)
   , _phi(0.0)
   , _stepWidth(0.0)
{
   printf("LBRTorqueSineOverlayClient initialized:\n"
         "\tjoint mask: 0x%x\n"
         "\tfrequency (Hz): %f\n"
         "\tamplitude (Nm): %f\n",
         jointMask, freqHz, torqueAmplitude);


   for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
}
*/
//******************************************************************************
SlidingModeClient::~SlidingModeClient()
{
}
      
//******************************************************************************
void SlidingModeClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // react on state change events
   switch (newState)
   {
      case MONITORING_WAIT:
      {
         for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}

         k_1 = 100;
         K_s = K_s * 1;
         ts = robotState().getSampleTime();          
         k_pinv_J = 0.001;
         alpha = 1;
         a_h = 0.2;                     
         a_r = 0.5;                      
         a_t = 0.8;
         a = 50;
         w_s = 0.0001;
         beta = M_PI/4;
         sigma = M_PI/2;
         lambda = 2;




         file_z.open( z_name );
         if (file_z.is_open()) cout << "file_z_opened" << endl;

         file_dz.open( dz_name );
         if (file_dz.is_open()) cout << "file_dz_opened" << endl;

         file_y.open( y_name );
         if (file_y.is_open()) cout << "file_y_opened" << endl;

         file_dy.open( dy_name );
         if (file_z.is_open()) cout << "file_dy_opened" << endl;

         double y_temp = 0;
         std::string line;

         while ( std::getline( file_z, line ) ) {
            y_temp = atof( line.c_str() );
            z_vec.push_back( y_temp );
         }
         while ( std::getline( file_dz, line ) ) {
            y_temp = atof( line.c_str() );
            dz_vec.push_back( y_temp );
         }  
         while ( std::getline( file_y, line ) ) {
            y_temp = atof( line.c_str() );
            y_vec.push_back( y_temp );
         }
         while ( std::getline( file_dy, line ) ) {
            y_temp = atof( line.c_str() );
            dy_vec.push_back( y_temp );
         }  


         file_z.close( );
         file_dz.close( );
         file_y.close( );
         file_dy.close( );

         count=0;

         x_i << 0.4, -y_vec[count] , z_vec[count], M_PI, 0, M_PI;

         controller = new REGION_CONTROLLER( x_i, a_h, a_r, a_t, alpha, k_1, ts, K_s, k_pinv_J, a,beta,sigma,w_s,lambda);


         file_tau_a.open(tau_a_name);
         file_tau_b.open(tau_b_name);
         file_tau.open(tau_name);
         file_M.open(M_name);
         file_A.open(A_name);
         file_w.open(w_name);
         file_s.open(s_name);
         file_kp.open(kp_name);
         file_xe.open(xe_name);
         file_Delta.open(Delta_name);
         file_ref.open(xref_name);
         file_q.open(q_name);
         file_sx.open(sx_name);
         file_dxe.open(dxe_name);
         file_dref.open(dxref_name);
         if (!file_tau.is_open()) cout << "tau_not_opened" << endl;
         if (!file_M.is_open()) cout << "M_not_opened" << endl;
         if (!file_w.is_open()) cout << "w_not_opened" << endl;
         if (!file_s.is_open()) cout << "s_not_opened" << endl;
         if (!file_kp.is_open()) cout << "kp_not_opened" << endl;
         if (!file_Delta.is_open())   cout << "Delta_not_opened" << endl;
         if (!file_xe.is_open())   cout << "xe_not_opened" << endl;
         if (!file_ref.is_open())   cout << "xerf_not_opened" << endl;
         if (!file_A.is_open()) cout << "A_not_opened" << endl;

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
}

//******************************************************************************
void SlidingModeClient::monitor()
{
   LBRClient::monitor();
   
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
   LBRClient::waitForCommand();
   
    /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/
   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is 
   // only necessary, that some torque values are sent. The LBR does not take the 
   // specific value into account.
   if (robotState().getClientCommandMode() == TORQUE)
   {
      robotCommand().setTorque(_torques);
   }
}

//******************************************************************************
void SlidingModeClient::command()
{

   // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    LBRClient::command();
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/   
   double Toruqe_Joint[KUKA::FRI::LBRState::NUMBER_OF_JOINTS]; 
   double Position_Joint[KUKA::FRI::LBRState::NUMBER_OF_JOINTS]; 

   // Check for correct ClientCommandMode.
   if (robotState().getClientCommandMode() == TORQUE)
   { 
       // calculate  offset
//      memcpy(Toruqe_Joint, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      
      memcpy(Position_Joint, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      
      robotCommand().setJointPosition(Position_Joint);
      
      for(int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
      { 
         q(i) = Position_Joint[i];
      }
      if(count > y_vec.size()-1) count=y_vec.size()-1;

      x_des << 0.4, -y_vec[count] , 0.614 , M_PI, 0, M_PI;//z_vec[count]

      x_des_dot << 0, -dy_vec[count] , 0.0 , 0.0, 0.0, 0.0;//dz_vec[count]

      count=count+1;


      controller->torque_control(q,x_des,x_des_dot,fm);

      file_q << controller-> get_dq().transpose() << endl;

      file_ref << x_des.transpose() << endl;

      file_xe << controller->get_x_e().transpose() << endl;

      file_M << controller->get_fh().transpose() << endl;

      file_w << controller->get_w() << endl;

      file_s << controller->get_q().transpose() << endl;

      file_kp << controller->get_kp() << endl;

      file_Delta << controller->get_Delta_x().transpose() << endl;

      file_A << controller->get_gamma() << endl;

      file_sx << controller->get_sx().transpose() << endl;

      file_dref << x_des_dot.transpose() << endl;

      file_dxe << controller->get_x_e_dot().transpose() << endl;


      file_tau_a << controller->get_tau_m_a().transpose() << endl;
      file_tau_b << controller->get_tau_m_b().transpose() << endl;


      tau = controller->get_tau();

      for (int i = 0; i < 7; ++i)
      {
         if (tau(i) > 40) tau(i)=40;
         if (tau(i) < -40) tau(i)=-40;
      }


      file_tau << controller->get_tau().transpose() << endl;


      for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
      {
         _torques[i] = tau(i);
      }


             
       // Set superposed joint torques.
      robotCommand().setTorque(_torques);
   }
}



// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif