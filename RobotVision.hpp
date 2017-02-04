#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "opencv2/opencv.hpp"
#include "GripPipeline.h"

#include <ntcore.h>
#include <networktables/NetworkTable.h>


typedef std::vector<cv::Point> shape;

class RobotVision{
private:
	static grip::GripPipeline *cam;

	static void VisionThread();

	static void drawHWC(cv::Mat &frame, std::vector<shape> &filterContoursOutput); // Height, Width, CenterX/Y

	static void drawArea(cv::Mat &frame, std::vector<shape> &filterContoursOutput);

	static std::shared_ptr<NetworkTable> contours;



public:
	static void cameraInit();
};