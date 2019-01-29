#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "opencv2/opencv.hpp"
#include <iomanip>
#include "GripPipeline.h"
#include "UDPSender.h"

#include <ntcore.h>

#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include <signal.h>


typedef std::vector<cv::Point> shape;

class RobotVision{
private:
	static grip::GripPipeline *cam;

	static void VisionThread();

	//static void drawHWC(cv::Mat &frame, std::vector<shape> &filterContoursOutput); // Height, Width, CenterX/Y
	//static void drawArea(cv::Mat &frame, std::vector<shape> &filterContoursOutput);
	static void drawHWCA(cv::Mat &frame, std::vector<shape> &filterContoursOutput,std::vector<grip::Line> &filterLinesOutput); // Height, Width, CenterX/Y, Area
	
	static std::vector<double> doubleVectorToIntVector(std::vector<int> in);

	// Used for UDP communications (new way of sending contours that took the place of NetworkTables)
	static UDPSender *udp;

public:
	static void cameraInit();
};
