#include <stdio.h>
#include "helper.h"

#define OUTPUT_PORT 9090

typedef struct skin_socket_struct {
	int skinType;
	int calibrationMethod;
	float tactileMap[12][12];
	float fx_modules[6][6];
	float fy_modules[6][6];
	float fz_modules[6][6];
	float contact_points[6][6][6];
	float fx_resultant[36]; 
	float fy_resultant[36]; 
	float fz_resultant[36];
	float compact_contact_points[36][2];
}skin_socket_struct;

int main() {
	int skin_socket_read;
	
	listener_socket(OUTPUT_PORT, &skin_socket_read);
	
	int slen, rlen; 
	sockaddr_in si_me, si_other;
	unsigned char buf[2048];
	skin_socket_struct skin_input;
	
	while( 1 ) {
		slen = sizeof( si_other );
		rlen = recvfrom( skin_socket_read, buf, 2048, 0, (struct sockaddr*)&si_other, (socklen_t*)&slen);
		std::cout<<"rlen: "<<rlen<<std::endl;
		if(rlen !=-1) {
			skin_input = *(( skin_socket_struct *) (buf ));
			
			std::cout << "SKIN TYPE: " << skin_input.skinType << std::endl;
			std::cout << "CALIBRATION METHOD: " << skin_input.calibrationMethod << std::endl;

			std::cout << "TACTILE MAP:" << std::endl;
			for (int i = 0; i < 12; i++) {
				for (int j = 0; j < 12; j++) {
					std::cout<<skin_input.tactileMap[i][j]<<" ";
				}
				std::cout<<std::endl;
			}

			std::cout << "fx:" << std::endl;
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					std::cout<<skin_input.fx_modules[i][j]<<" ";
				}
				std::cout<<std::endl;
			}

			std::cout << "Contact Point:" << std::endl;
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
					std::cout<<skin_input.contact_point[i][j][0]<<" "<<skin_input.contact_point[i][j][1]<<" "<<skin_input.contact_point[i][j][2]<<" "<<skin_input.contact_point[i][j][3]<<" "<<skin_input.contact_point[i][j][4]<<" "<<skin_input.contact_point[i][j][5]<<std::endl;
				}
			}
		}
	}
}

