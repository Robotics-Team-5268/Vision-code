// Raspberry Pi
// IMPORTANT: Must compile with C++11 support for to_string to work!

// To compile: g++ -o Client Client.cpp -std=c++11
// That last part makes to_string work

//#define hostname "roboRIO-5268-FRC.local"
#define hostname "127.0.0.1"

#include "UDPSender.h"
using namespace std;

UDPSender::UDPSender() {
	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	serv.sin_family = AF_INET;
	serv.sin_port = htons(53000);
	serv.sin_addr.s_addr = inet_addr(hostname);

	// Testing
	vector<int> centerX = {562, 73, 147};
	vector<int> centerY = {412, 2, 90};
	vector<int> width = {365, 190, 42};
	vector<int> height = {52, 202, 73};
	vector<int> area = {1095, 1159, 2006};
	sendContours(centerX, centerY, width, height, area);
}

void UDPSender::sendContours(vector<int> centerX, vector<int> centerY, vector<int> width, vector<int> height, vector<int> area) {
	int numContours = sizeof(area)/sizeof(area[0]);
	string msg = "";
	for (int i = 0; i < numContours; i++) {
		
		msg  += to_string(centerX[i]) + "," 
		+ to_string(centerY[i]) + "," 
		+ to_string(width[i]) + "," 
		+ to_string(height[i]) + "," 
		+ to_string(area[i]);
		if (i < numContours - 1) { // Not the last contour
			msg += ";";
		}

	}
	sendto(sockfd, msg.c_str(), msg.size(), 0, (struct sockaddr*) &serv, sizeof(serv));
}