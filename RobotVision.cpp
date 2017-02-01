#include "RobotVision.hpp"
#include <sstream>

#define xres 720
#define yres 480

#define debug

grip::GripPipeline* RobotVision::cam = nullptr;

void RobotVision::VisionThread() {
	cam = new grip::GripPipeline();
	cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, xres);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, yres);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return;
    cv::Mat* frame = new cv::Mat();
    cv::Mat* hsvfilter = new cv::Mat();
    std::vector<std::vector<cv::Point> >* findContoursOutput = new std::vector<std::vector<cv::Point> >();
    for(int camCount=0;;camCount++){
        cap >> *frame;
        if( frame->empty() ) break; // end of video stream
		//cam->setsource0(frame);
        cam->process(*frame);
        findContoursOutput = cam->getfilterContoursOutput();
        for(std::vector<cv::Point> shape : *findContoursOutput){
            cv::polylines (*frame, *findContoursOutput, true, cv::Scalar(0xFF,0xFF,0xFF));
        }
#ifdef debug
        std::stringstream outputStream, filterSteam;
        outputStream << "output/output_" << camCount << ".png";
        filterSteam << "output/filter_" << camCount << ".png";
        imwrite(outputStream.str(), *frame);
        //imwrite(filterSteam.str(), *hsvfilter);
#else
        imshow("output", *frame);
        //imshow("filter", *hsvfilter);
        if( cv::waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
#endif
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