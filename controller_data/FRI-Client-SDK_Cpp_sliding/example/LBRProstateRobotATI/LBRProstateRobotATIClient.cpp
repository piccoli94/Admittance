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
#include <cstring>
#include <cstdio>
#include "LBRProstateRobotATIClient.h"
// Visual studio needs extra define to use math constants
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;


void LBRJointSineOverlayClient::forward_kinematics()
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
   
   


}


void LBRJointSineOverlayClient::ATI_data_cb()
{
	Matrix<double, N_samp ,1> f_temp_x;
   	Matrix<double, N_samp ,1> f_temp_y;
	Matrix<double, N_samp ,1> f_temp_z;

	uint8_t count = 0;
	float sum_f_x = 0;
	float sum_f_y = 0;
	float sum_f_z = 0;

	f_temp_x.setZero();
	f_temp_y.setZero();
	f_temp_z.setZero();

	while(1) 
	{
		/* Read ATI data */
		slen = sizeof( si_other );
		rlen = recvfrom( ati_socket_read, buf, sizeof(ATIdata_struct), 0, (struct sockaddr*)&si_other, (socklen_t*)&slen);

		if(rlen !=-1) 
		{
			ATI_input = *(( ATIdata_struct *) (buf));
			if( fabs(ATI_input.fz) > 3 )
			{
				fm  <<ATI_input.fx, ATI_input.fy, ATI_input.fz;
            fm -= force_bias;
			}
			else
			{
				fm <<0, 0, 0;
			}
		}
		else 
		{
			cout << "ATI no data available" << endl;
			fm  <<0, 0, 0;
		}
		
		/* moving average filter */
		f_temp_x[count] = fm[0];
      	f_temp_y[count] = fm[1];
      	f_temp_z[count] = fm[2];
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
		}	
		fm[0] = sum_f_x / N_samp;
      	fm[1] = sum_f_y / N_samp;
      	fm[2] = sum_f_z / N_samp;
		sum_f_x = 0;
		sum_f_y = 0;
		sum_f_z = 0;
		fm = Rbs * fm;
		std::cout << " fx: " << fm[0] << " fy: " << fm[1] << " fz: "<< fm[2] << std::endl;
}
}

//******************************************************************************
LBRJointSineOverlayClient::LBRJointSineOverlayClient(double a) 
{
    printf("LBRJointSineOverlayClient initialized: %d\n", a);
    bool socketIsValid = listener_socket(OUTPUT_PORT, &ati_socket_read);

	if(socketIsValid == false)
	{
		exit(-1);	
	}
	else
	{
		cout << "Communication socket with ATI sensor opened." << endl;

	}


	system("rosservice call /ft_sensor/bias_cmd \"cmd: 'bias'\"");
}

//******************************************************************************
LBRJointSineOverlayClient::~LBRJointSineOverlayClient()
{
}
//******************************************************************************

void LBRJointSineOverlayClient::onStateChange()
{
 		// Reset time axis
		time = 0.0;
		hz = 100.0;
		time_threshold_count = 0;

		// Reset force bias
		force_bias.setZero();
/*		q <<  0 ,0 ,0 ,-0.57 ,0 ,1.57 ,0;
		forward_kinematics();
		std::cout << " pex: " << Tbe(0,3)<< " pey: " << Tbe(1,3)<< " pez: " << Tbe(2,3) << std::endl;
*/		
		q << 0 ,0 ,0 ,-1.57 ,0 ,1.57 ,0;
		forward_kinematics();


/*		Res << 0, 1,  0,
							 			 1, 0,  0,
										 0, 0, -1;
*/
/*		Res << -1,  0, 0,
											0, -1, 0,
											0,  0, 1;

*/
		Res << 	1,  0, 0,
				0,  1, 0,
				0,  0, 1;
		Rbs= Rbe*Res;
		// Questo parametro influenza la pinv DLS e riduce le velocit� massime di giunto
		// Il parametro di ingresso NON � una vera saturazione ma � euristicamente legato alle massime velocit� di giunto che usciranno dalla pinv_DLS
		// attualente non esiste una relazione in forma chiusa per legare questo parametro alle vere massime velocit� di giunto che usciranno dalla pinv
		// ovviamente questo parametro pu� anche influenzare la stabilit� del clik
		// deve essere diverso da 0

}
   
//******************************************************************************
void LBRJointSineOverlayClient::command()
{

}
//******************************************************************************
// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
