#include "RobotVision.hpp"

#define xres 720
#define yres 480

//#define debug

grip::GripPipeline* RobotVision::cam = nullptr;

void RobotVision::VisionThread() {
	cam = new grip::GripPipeline::GripPipeline();
	cv::VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return;
    cv::Mat frame;
    for(;;){
        cap >> frame;
        if( frame.empty() ) break; // end of video stream
		//cam->setsource0(frame);
        cam->process(frame);
        cv::rectangle(frame, cv::Point(100,100), cv::Point(400,400), cv::Scalar(0xFF,0xFF,0xFF),5);
        imshow("output", frame);
        if( cv::waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
        std::cout << "findContours found: " << cam->getfindContoursOutput()->size() << " Contours found: " << cam->getfilterContoursOutput()->size() << std::endl;
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return;
}

void RobotVision::cameraInit() {
	// We need to run our vision program in a separate Thread.
	// If not, our robot program will not run

	//std::thread visionThread(VisionThread);
	//visionThread.detach();
	VisionThread();
}

int main(){
	RobotVision::cameraInit();
}