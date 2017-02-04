#include "RobotVision.hpp"

#define xres 720
#define yres 480

//#define hostname "10.7.181.32"
//#define hostname "10.0.0.99"
#define hostname "roboRIO-5268-FRC.local"

//#define debug
//#define noCam

#ifdef debug
#define saveImages
#endif

grip::GripPipeline *RobotVision::cam = nullptr;
std::shared_ptr<NetworkTable> RobotVision::contours = nullptr;

void RobotVision::VisionThread(){
	cam = new grip::GripPipeline();
#ifndef noCam
	cv::VideoCapture cap;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, xres);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, yres);
#endif
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress(hostname);
	NetworkTable::Initialize();
	NetworkTable::GlobalDeleteAll();
	contours = NetworkTable::GetTable("Contours");
#ifndef noCam
	if(!cap.open(1))
		return;
	cv::Mat *frame = new cv::Mat();
#else
	cv::Mat frameData = cv::imread ("../output/output_3.png");
	cv::Mat* frame = &frameData;
#endif

	std::vector<shape> *filterContoursOutput;
	for(;;){
#ifndef noCam
		cap >> *frame;
		if(frame->empty()) break; // end of video stream
#endif
		//cam->setsource0(frame);
		cam->process(*frame);
		filterContoursOutput = cam->getfilterContoursOutput();
		cv::polylines(*frame, *filterContoursOutput, true, cv::Scalar(0xFF, 0xFF, 0xFF, 0x7F));
		drawCenters(*frame, *filterContoursOutput);
#ifdef saveImages
		std::stringstream outputStream, filterSteam;
		outputStream << "../output/output_" << camCount << ".png";
		filterSteam << "../output/filter_" << camCount << ".png";
		imwrite(outputStream.str(), *frame);
#endif
#ifdef debug
		imshow("output", *frame);
		if(cv::waitKey(10) == 27) break; // stop capturing by pressing ESC
#endif
		contours->PutNumber("NumberOfContours", filterContoursOutput->size());
#ifdef debug
		std::cout << "findContours found: " << cam->getfindContoursOutput()->size() << " Contours found: "
		          << filterContoursOutput->size() << std::endl;
#endif
	}
	// the camera will be closed automatically upon exit
	// cap.close();
	return;
}

void RobotVision::cameraInit(){
	// We need to run our vision program in a separate Thread.
	// If not, our robot program will not run

	//std::thread visionThread(VisionThread);
	//visionThread.detach();
	VisionThread();
}

void RobotVision::drawCenters(cv::Mat &frame, std::vector<shape> &filterContoursOutput){
	if(!filterContoursOutput.size()) return; // if empty do nothing

	int runtimes = 0;
	for(shape shapes : filterContoursOutput){
		cv::Point center;
		std::stringstream output;
		int numOfPoints = 0;
		for(;numOfPoints < shapes.size();numOfPoints++){
			cv::Point *point = &shapes[numOfPoints];
			center.x += point->x;
			center.y += point->y;
		}
		center.x /= numOfPoints;
		center.y /= numOfPoints;
		std::vector<double> values = {(double)center.x, (double)center.y};
		output << "CenterPoint" << runtimes;
		runtimes++;
		contours->PutNumberArray(output.str(), values);
		cv::line(frame, center, center, cv::Scalar(0xFF, 0x00, 0x00, 0x7F));
	}
}

int main(){
	RobotVision::cameraInit();
}