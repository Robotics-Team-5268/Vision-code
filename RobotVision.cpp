#include "RobotVision.hpp"
#include "UDPSender.h"
#include "GripPipeline.h"

#define xres 320 // Should be 640...?
#define yres 240

// Exposure defined in cameraInit
// From 5 to 20,000 (although it seems to max out long before that)

#define pidPath "/tmp/Vision-Code.pid"

#define debug
//#define saveImages

#ifdef debug
//#define logToFile
#define displayVideo
//#define noCam
#define saveImages
//#define logContours
#endif

#ifdef saveImages
#define padAmounts 10
//#define saveOnlyOnContours
#endif

#ifdef logToFile
#define log logFile
std::ofstream logFile("vision.log");
#else
#define log std::cout
#endif

grip::GripPipeline *RobotVision::cam = nullptr;
UDPSender *RobotVision::udp = nullptr; // It won't find the variable from the header file otherwise for some reason

#ifdef debug
cv::Mat *orig = new cv::Mat();
#endif

void RobotVision::VisionThread(){
	cam = new grip::GripPipeline();
#ifndef noCam
	cv::VideoCapture cap;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, xres);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, yres);
	cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0); // This does not work
	cap.set(cv::CAP_PROP_EXPOSURE, -8.0); // This does not work
#endif
	udp = new UDPSender();

#ifndef noCam
	if(!cap.open(0))
		return;
	cv::Mat *frame = new cv::Mat();
#else
	cv::Mat frameData = cv::imread("../output/output_3.png");
	cv::Mat* frame;
#endif

	std::vector<shape> *filterContoursOutput;
	std::vector<grip::Line> *filterLinesOutput;
#ifdef saveImages
	long camCount = 0;
#endif
	while(true) {
#ifndef noCam
		//int frameCount = cap.get(CV_CAP_PROP_FRAME_COUNT); Not supported with v4l2 or our camera
		//cap.set(CV_CAP_PROP_POS_FRAMES, frameCount - 1); Not supported with v4l2 or our camera
		cap >> *frame;
#ifdef debug
		frame->copyTo(*orig);
#endif
		if(frame->empty()){
			log << "No camera" << std::endl;
			break; // end of video stream
		}
#else
		frame = &frameData;
#endif
		cam->Process(*frame);
		filterContoursOutput = cam->GetFilterContoursOutput();
		filterLinesOutput = cam->GetFilterLinesOutput();
#ifdef saveImages
#ifdef saveOnlyOnContours
		if(!filterContoursOutput->size() || !getFilterLinesOutput->size()){
			continue;
		}
#endif
		std::stringstream outputStream, overlayStream;
		outputStream << "../output/output_" << std::setfill('0') << std::setw(padAmounts) << camCount << ".png";
		overlayStream << "../output/overlay_" << std::setfill('0') << std::setw(padAmounts) << camCount << ".png";
		imwrite(outputStream.str(), *frame);
#endif
		//drawHWCA(*frame, *filterContoursOutput, *filterLinesOutput);
#ifdef debug
		cv::polylines(*frame, *cam->GetFindContoursOutput(), true, cv::Scalar(0xFF, 0x00, 0x00, 0x7F));
		cv::polylines(*frame, *filterContoursOutput, true, cv::Scalar(0x00, 0x00, 0xFF, 0x7F));
#endif
#ifdef debug
		std::stringstream contours;
		contours << "findContours num: " << cam->GetFindContoursOutput()->size() << " filterContours num: "
		         << filterContoursOutput->size();
		cv::putText(*frame, contours.str(), cv::Point(0,30),
		            cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0x00, 0xFF, 0x00));
#ifdef logContours
		log << contours.str() << std::endl;
#endif
#endif
#ifdef saveImages
		imwrite(overlayStream.str(), *frame);
		camCount++;
#endif
#ifdef displayVideo
		imshow("output", *frame);
		if(cv::waitKey(10) == 27) break; // stop capturing by pressing ESC
#endif
	}
	// the camera will be closed automatically upon exit
	// cap.close();
}

// Effectively the constructor
void RobotVision::cameraInit(){
	// Set up camera exposure
	system("v4l2-ctl -d /dev/video0 -c exposure_auto=1");
	system("v4l2-ctl -d /dev/video0 -c exposure_absolute=20");
	
	//std::thread visionThread(VisionThread);
	//visionThread.detach();
	VisionThread();
}

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
#ifdef debug
		const int scale = 5;
		cv::Vec3b color = orig->at<cv::Vec3b>(cv::Point((int)x, (int)y));
		cv::line(frame,
		         cv::Point((int) x, (int) y),
		         cv::Point((int) x, (int) y),
		         color,
		         scale);
#endif
	}
	for (grip::Line lines : filterLinesOutput) {
		x1.push_back((int)lines.x1);
		y1.push_back((int)lines.y1);
		x2.push_back((int)lines.x2);
		y2.push_back((int)lines.y2);
		angle.push_back((int) lines.angle());
	}
	
	udpsender->sendContours(centerX, centerY, width, height, x1, y1, x2, y2, angle);
	
#ifdef debug
	cv::drawContours(frame, filterContoursOutput, -1, cv::Scalar(0xFF, 0x00, 0x00, 0xFF/4), CV_FILLED);
#endif
}

inline bool exists(const std::string& name) {
	struct stat buffer;
	return stat(name.c_str(), &buffer) == 0;
}

int old_main(){
	pid_t pid;
	if(exists(pidPath)){
		log << "PID file exists, checking if process exists" << std::endl;
		std::fstream pidFile(pidPath, std::fstream::in);
		pidFile >> pid;
		struct stat sts;
		if (kill(pid, 0) == 0) {
			log << "process exists, killing" << std::endl;
			std::stringstream cmd; // process does exist
			cmd << "kill -9 " << pid;
			system(cmd.str().c_str());
		} else {
			log << "process doesn't exit, ignoring" << std::endl;
		}
		pid = getpid();
		pidFile.close();
		pidFile.open(pidPath, std::fstream::out | std::fstream::trunc);
		pidFile << pid;
		pidFile.close();
		sleep(1); //wait for process to exit
	} else {
		log << "No PID File, creating" << std::endl;
		std::fstream pidFile(pidPath, std::fstream::out);
		pidFile << getpid();
		pidFile.close();
	}
	RobotVision::cameraInit();
}
