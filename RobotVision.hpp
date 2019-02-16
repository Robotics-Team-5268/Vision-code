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
	static std::vector<double> doubleVectorToIntVector(std::vector<int> in);

public:
	static void drawHWCA(cv::Mat &frame, std::vector<shape> &filterContoursOutput,std::vector<grip::Line> &filterLinesOutput, UDPSender *udpsender); // Height, Width, CenterX/Y, Area
	
};
