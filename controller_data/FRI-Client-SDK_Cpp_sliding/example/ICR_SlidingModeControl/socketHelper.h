/*
 *	Funzioni di supporto per l'esecuzione del nodo di controllo
 *
 * 	Author: Prisma Lab <jonathan.cacace@gmail.com>
 *   
 */
#ifndef SOCKET_HELPER
#define SOCKET_HELPER

#include <stdio.h>
#include <string.h>
#include <iostream>
//Socket functions
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>

//Creazione socket in LETTURA
bool listener_socket(int port_number, int *sock);

//Creazione socket in SCRITTURA
int create_socket(char* dest, int port, int *sock);


#endif
