#include "socketHelper.h"

//Creazione socket in LETTURA
bool listener_socket(int port_number, int *sock) {
	sockaddr_in si_me, si_other;

	if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		std::cout << "Listener::Open: error during socket creation!" << std::endl;
		return false;
	}

	memset((char *) &si_me, 0, sizeof(si_me));

	/* allow connections to any address port */
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port_number);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me))==-1) {
		std::cout << "Listener::Open: error during bind!" << std::endl;
		return false;
	}
	return true;
}

//Creazione socket in SCRITTURA
int create_socket(char* dest, int port, int *sock) {
	struct sockaddr_in temp; 
	struct hostent *h;
	int error;

	temp.sin_family=AF_INET;
	temp.sin_port=htons(port);
	h=gethostbyname(dest);

	if (h==0) {
		printf("Gethostbyname fallito\n");
		//exit(1);
	}

	bcopy(h->h_addr,&temp.sin_addr,h->h_length);
	*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
	return error;
}
