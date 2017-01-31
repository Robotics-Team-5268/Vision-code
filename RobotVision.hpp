#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "opencv2/opencv.hpp"

#include "GripPipeline.cpp"

class RobotVision {
private:
	static grip::GripPipeline* cam;
	static void VisionThread();
public:
	static void cameraInit();
};