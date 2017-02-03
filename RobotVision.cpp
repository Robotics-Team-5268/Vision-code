#include "RobotVision.hpp"
#include <sstream>

#include <ntcore.h>
#include <networktables/NetworkTable.h>

#define xres 720
#define yres 480

//#define hostname "10.7.181.32"
#define hostname "10.0.0.99"

//#define debug
#define noCam

typedef std::vector<cv::Point> shape;

grip::GripPipeline* RobotVision::cam = nullptr;

void RobotVision::VisionThread() {
	cam = new grip::GripPipeline();
#ifndef noCam
	cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, xres);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, yres);
#endif
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    NetworkTable::SetClientMode();
    NetworkTable::SetIPAddress(hostname);
    NetworkTable::Initialize();
    std::shared_ptr<NetworkTable> myTable = NetworkTable::GetTable("Contours");
#ifndef noCam
    if(!cap.open(0))
        return;
    cv::Mat* frame = new cv::Mat();
#else
    cv::Mat frameData = cv::imread ("../output/output_3.png");
    cv::Mat* frame = &frameData;
#endif

    std::vector<shape>* filterContoursOutput = new std::vector<shape>();
    for(int camCount=0;;camCount++){
#ifndef noCam
        cap >> *frame;
        if( frame->empty() ) break; // end of video stream
#endif
		//cam->setsource0(frame);
        cam->process(*frame);
        filterContoursOutput = cam->getfilterContoursOutput();
        cv::polylines (*frame, *filterContoursOutput, true, cv::Scalar(0xFF,0xFF,0xFF));
#ifdef debug
        std::stringstream outputStream, filterSteam;
        outputStream << "../output/output_" << camCount << ".png";
        filterSteam << "../output/filter_" << camCount << ".png";
        imwrite(outputStream.str(), *frame);
#else
        imshow("output", *frame);
        if( cv::waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
#endif
        std::stringstream numberOfContours;
        numberOfContours << filterContoursOutput->size();
        myTable->PutString("NumberOfContours", numberOfContours.str());
        std::cout << "findContours found: " << cam->getfindContoursOutput()->size() << " Contours found: " << filterContoursOutput->size() << std::endl;
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