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
#ifndef _KUKA_FRI_LBR_JOINT_SINE_OVERLAY_CLIENT_H
#define _KUKA_FRI_LBR_JOINT_SINE_OVERLAY_CLIENT_H


/* Include SUN-Robot LIB */

#include <vector>
#include "socketHelper.h"	
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

/* Configuration for ATI sensor */
#define OUTPUT_PORT 9090

/* Admittance Control parameters */
#define F_THRESHOLD 2 /*[N]*/
#define TIME_THRESHOLD 5 /*[s]*/
#define DESIRED_FORCE -2 /*[N]*/

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

using namespace Eigen;
/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class LBRJointSineOverlayClient
{
   
public:
      
   /**
    * \brief Constructor.
    */
   LBRJointSineOverlayClient(double a);
   
   /** 
    * \brief Destructor.
    */
   ~LBRJointSineOverlayClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();
	
	void ATI_data_cb();

	void forward_kinematics();
      
private:


	double time;			/* Time axis */
	double hz;				/* Frequency value */

	Eigen::Matrix<double, 3 ,1> fm; /* [N] */	
Eigen::Matrix<double, 3 ,1> force_bias;
	
	/* Trajectory object for one segment */
		      

	/* Support variables for ATI communication */
	int ati_socket_read;
	int slen;
	int rlen; 
	sockaddr_in si_me, si_other;
	unsigned char buf[24];
	ATIdata_struct ATI_input;
	float time_threshold_count;

	Matrix<double,3,3> Rbe;
	Matrix<double,4,4> Tbe;
	Matrix<double,3,3> Res;
	Matrix<double,3,3> Rbs;
	Matrix<double,7,1> q;
	double db=0.0; 
   	double d0=0.34; 
   	double d3=0.40; 
   	double d5=0.40; 
   	double d7=0.126;

	/* PointMap variables */
	uint16_t NUM_CART_POINTS;


};

#endif // _KUKA_FRI_LBR_JOINT_SINE_OVERLAY_CLIENT_H
