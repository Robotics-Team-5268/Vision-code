#include "RobotVision.hpp"

#define xres 720
#define yres 480

//#define hostname "10.7.181.32"
//#define hostname "10.0.0.99"
#define hostname "roboRIO-5268-FRC.local"

//#define debug

#ifdef debug
//#define noCam
//#define saveImages
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
	contours = NetworkTable::GetTable("roboRIO-5268-frc.local");
#ifndef noCam
	if(!cap.open(0))
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
		drawHWC(*frame, *filterContoursOutput);
		drawArea(*frame, *filterContoursOutput);
#ifdef saveImages
		std::stringstream outputStream, filterSteam;
		outputStream << "../output/output_" << camCount << ".png";
		filterSteam << "../output/filter_" << camCount << ".png";
		imwrite(outputStream.str(), *frame);
#endif
#ifdef debug
		imshow("output", *frame);
		if(cv::waitKey(10) == 27) break; // stop capturing by pressing ESC
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

void RobotVision::drawHWC(cv::Mat &frame, std::vector<shape> &filterContoursOutput){
	std::vector<double> height;
	std::vector<double> width;
	std::vector<double> centerX;
	std::vector<double> centerY;
	for(shape shapes : filterContoursOutput){
		cv::Rect boundingBox = cv::boundingRect(shapes);
		//cv::convexHull()
		height.push_back(0+boundingBox.height);
		width.push_back(0+boundingBox.width);
		double x,y;
		x = boundingBox.x + boundingBox.width / 2;
		y = boundingBox.y + boundingBox.height / 2;
		centerX.push_back(x);
		centerY.push_back(y);
#ifdef debug
		const int scale = 5;
		cv::line(frame, cv::Point((int)x, (int)y), cv::Point((int)x, (int)y), cv::Scalar(0xFF, 0x00, 0x00, 0x7F), scale);
#endif
	}
	contours->PutNumberArray("height", height);
	contours->PutNumberArray("width", width);
	contours->PutNumberArray("centerX", centerX);
	contours->PutNumberArray("centerY", centerY);
}

void RobotVision::drawArea(cv::Mat &frame, std::vector<shape> &filterContoursOutput){
	std::vector<double> areas;
	for(shape shapes : filterContoursOutput){
		areas.push_back(cv::contourArea(shapes));
	}
	contours->PutNumberArray("area", areas);
#ifdef debug // todo

#endif
}

int main(){
	RobotVision::cameraInit();
}
