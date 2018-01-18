#pragma once
#include <vector>
#include <sys/socket.h>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>

class UDPSender {

private:
	int sockfd;
	struct sockaddr_in serv;
	
public:
	UDPSender();
	void sendContours(std::vector<int> centerX, std::vector<int> centerY, std::vector<int> width, std::vector<int> height, std::vector<int> area);
};