#include "RobotVision.hpp"
#include "UDPSender.h"
#include "GripPipeline.h"

void RobotVision::drawHWCA(cv::Mat &frame, std::vector<shape> &filterContoursOutput,std::vector<grip::Line> &filterLinesOutput, UDPSender *udpsender) { 
	std::vector<int> height;
	std::vector<int> width;
	std::vector<int> centerX;
	std::vector<int> centerY;
	std::vector<int> x1;
	std::vector<int> y1;
	std::vector<int> x2;
	std::vector<int> y2;
	std::vector<int> angle;
	
	for(shape shapes : filterContoursOutput){
		cv::Rect boundingBox = cv::boundingRect(shapes);
		//cv::convexHull()
		height.push_back(0 + boundingBox.height);
		width.push_back(0 + boundingBox.width);
		int x, y;
		x = boundingBox.x + boundingBox.width / 2;
		y = boundingBox.y + boundingBox.height / 2;
		centerX.push_back(x);
		centerY.push_back(y);
	}
	for (grip::Line lines : filterLinesOutput) {
		x1.push_back((int)lines.x1);
		y1.push_back((int)lines.y1);
		x2.push_back((int)lines.x2);
		y2.push_back((int)lines.y2);
		angle.push_back((int) lines.angle());
	}
	
	udpsender->sendContours(centerX, centerY, width, height, x1, y1, x2, y2, angle);
	
}
