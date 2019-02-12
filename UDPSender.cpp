// Raspberry Pi
// IMPORTANT: Must compile with C++11 support for to_string to work!

//#define hostname "roboRIO-5268-FRC.frc-robot.local"
//#define hostname "roboRIO-5268-FRC.local"
#define hostname "localhost"
//#define hostname "biomechfalcons-HP-G60-Notebook-PC"

#define port 5805
#define debugToStdOut

#ifdef debugToStdOut
#include <iostream>
#endif

#include "UDPSender.h"
using namespace std;

UDPSender::UDPSender() {
	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	serv.sin_family = AF_INET;
	serv.sin_port = htons(port);
	
	hostent* record = gethostbyname(hostname);
	in_addr* address = (in_addr*) record->h_addr;
	const char* ipaddress = inet_ntoa(*address);
	// This MUST be given an ip address, NOT a hostname
	serv.sin_addr.s_addr = inet_addr(ipaddress); // "10.52.68.71"
}

void UDPSender::sendContours(std::vector<int> centerX, std::vector<int> centerY, std::vector<int> width, std::vector<int> height, 
	std::vector<int> x1, std::vector<int> y1, std::vector<int> x2, std::vector<int> y2, std::vector<int> angle) {
	int numContours = centerX.size();
	string msg = "";
	for (int i = 0; i < numContours; i++) {
		
		msg  += to_string(centerX[i]) + "," 
		+ to_string(centerY[i]) + "," 
		+ to_string(width[i]) + "," 
		+ to_string(height[i]) + ","
		+ to_string(x1[i]) + ","
		+ to_string(y1[i]) + ","
		+ to_string(x2[i]) + ","
		+ to_string(y2[i]) + ","
		+ to_string(angle[i]);
		if (i < numContours - 1) { // Not the last contour
			msg += ";";
		}

	}
	#ifdef debugToStdOut
	std::cout << msg << std::endl;
	#endif
	// sizeof(serv)+1 tells it to send one extra character - the null pointer after the characters
	sendto(sockfd, msg.c_str(), msg.size()+1, 0, (struct sockaddr*) &serv, sizeof(serv));
}
