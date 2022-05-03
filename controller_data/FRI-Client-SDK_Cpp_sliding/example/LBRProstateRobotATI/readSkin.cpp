#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include "skin.h"
#include <sys/time.h>

using namespace std;

/* Check if a keyboard button has been pressed.*/
int kbhit(void) 
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

int main() {

	
	Skin skin1(RIGID_SKIN, NOT_FINE_CALIBRATION, "127.0.0.1", 9090); /* Create a Skin object.*/

	struct timeval tp;
	double sec, usec, start, end;
 
	skin1.setCOM("/dev/ttyACM0"); /* Set the Serial Communication Port.*/
	skin1.connect(); /* Connect to the Skin communication port.*/

	skin1.removeOffset(20); /* Remove the offset from the voltage signals.*/
	
	while (1) {

		gettimeofday( &tp, NULL );
		sec = static_cast<double>( tp.tv_sec );
		usec = static_cast<double>( tp.tv_usec )/1E6;
		start = sec + usec;

		skin1.update();

		gettimeofday( &tp, NULL );
		sec = static_cast<double>( tp.tv_sec );
		usec = static_cast<double>( tp.tv_usec )/1E6;
		end = sec + usec;

		double time = end - start;
		//cout << time << "\r\n";

		if(kbhit()!=0) /* If a keyboard button has been pressed the sensor reading is killed.*/
		{
			break;
		}
	}

	skin1.disconnect(); /* Close the sensor communication.*/

	return 0;
}
