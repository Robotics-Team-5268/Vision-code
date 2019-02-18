#pragma once
#include <vector>
#include <sys/socket.h>
#include <iostream>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include "Contour.h"

class UDPSender {

private:
	int sockfd;
	struct sockaddr_in serv;
	
public:
	UDPSender();
	void sendContours( const std::vector<Contour>& aContours );
};
