#include "RobotVision.hpp"

#define xres 720
#define yres 480

//#define hostname "10.7.181.32"
//#define hostname "10.0.0.99"
#define hostname "roboRIO-5268-FRC.local"
//#define hostname "localhost"

#define pidPath "/tmp/Vision-Code.pid"

//#define debug

#ifdef debug
//#define logToFile
#define displayVideo
//#define noCam
//#define saveImages
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
std::shared_ptr<NetworkTable> RobotVision::contours = nullptr;
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
	cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
	cap.set(cv::CAP_PROP_EXPOSURE, -8.0);
#endif
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress(hostname);
	NetworkTable::Initialize();
	NetworkTable::GlobalDeleteAll();
	contours = NetworkTable::GetTable("roboRIO-5268-frc.local");
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
#ifdef saveImages
	long camCount = 0;
#endif
	while(true) {
#ifndef noCam
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
		cam->process(*frame);
		filterContoursOutput = cam->getfilterContoursOutput();
#ifdef saveImages
#ifdef saveOnlyOnContours
		if(!filterContoursOutput->size()){
			continue;
		}
#endif
		std::stringstream outputStream, overlayStream;
		outputStream << "../output/output_" << std::setfill('0') << std::setw(padAmounts) << camCount << ".png";
		overlayStream << "../output/overlay_" << std::setfill('0') << std::setw(padAmounts) << camCount << ".png";
		imwrite(outputStream.str(), *frame);
#endif
		//drawArea(*frame, *filterContoursOutput);
		//drawHWC(*frame, *filterContoursOutput);
		drawHWCA(*frame, *filterContoursOutput);
#ifdef debug
		cv::polylines(*frame, *cam->getfindContoursOutput(), true, cv::Scalar(0xFF, 0x00, 0x00, 0x7F));
		cv::polylines(*frame, *filterContoursOutput, true, cv::Scalar(0x00, 0x00, 0xFF, 0x7F));
#endif
#ifdef debug
		std::stringstream contours;
		contours << "findContours found: " << cam->getfindContoursOutput()->size() << " Contours found: "
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
//std::thread visionThread(VisionThread);
	//visionThread.detach();
	
	VisionThread();
}

void RobotVision::drawHWCA(cv::Mat &frame, std::vector<shape> &filterContoursOutput){
	std::vector<int> height;
	std::vector<int> width;
	std::vector<int> centerX;
	std::vector<int> centerY;
	std::vector<int> area;
	
	for(shape shapes : filterContoursOutput){
		area.push_back(cv::contourArea(shapes));
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
	
	// Network table code - remove later
	std::vector<double> height_d = doubleVectorToIntVector(height);
	contours->PutNumberArray("height", height_d);
	std::vector<double> width_d = doubleVectorToIntVector(width);
	contours->PutNumberArray("width", width_d);
	std::vector<double> centerX_d = doubleVectorToIntVector(centerX);
	contours->PutNumberArray("centerX", centerX_d);
	std::vector<double> centerY_d = doubleVectorToIntVector(centerY);
	contours->PutNumberArray("centerY", centerY_d);
	std::vector<double> area_d = doubleVectorToIntVector(area);
	contours->PutNumberArray("area", area_d);
	
	udp->sendContours(centerX, centerY, width, height, area);
	
	// Test data
	/*std::vector<int> centerX_test = {562, 73, 147};
	std::vector<int> centerY_test = {412, 2, 90};
	std::vector<int> width_test = {365, 190, 42};
	std::vector<int> height_test = {52, 202, 73};
	std::vector<int> area_test = {1095, 1159, 2006};
	udp->sendContours(centerX_test, centerY_test, width_test, height_test, area_test);*/
	
	/*std::vector<int> centerX_test;
	std::vector<int> centerY_test;
	std::vector<int> width_test;
	std::vector<int> height_test;
	std::vector<int> area_test;
	udp->sendContours(centerX_test, centerY_test, width_test, height_test, area_test);*/
	
#ifdef debug
	cv::drawContours(frame, filterContoursOutput, -1, cv::Scalar(0xFF, 0x00, 0x00, 0xFF/4), CV_FILLED);
#endif
}

// Temporary, to support network tables (which need doubles) alongside int-based UDP protocol
std::vector<double> RobotVision::doubleVectorToIntVector(std::vector<int> in) {
	std::vector<double> out;
	for (int value : in) {
		out.push_back( (double) value );
	}
	return out;
}

inline bool exists(const std::string& name) {
	struct stat buffer;
	return stat(name.c_str(), &buffer) == 0;
}

int main(){
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
