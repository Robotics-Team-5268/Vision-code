#include "GripPipeline.h"
/**
* Initializes a GripPipeline.
*/

namespace grip {

GripPipeline::GripPipeline() {
}
/**
* Runs an iteration of the Pipeline and updates outputs.
*
* Sources need to be set before calling this method. 
*
*/
void GripPipeline::process(cv::Mat source0){
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThresholdInput = source0;
	double hsvThresholdHue[] = {0.0, 100.13651877133107};
	double hsvThresholdSaturation[] = {0.0, 20.017064846416375};
	double hsvThresholdValue[] = {200.955035971223, 255.0};
	std::cout << "  Calling hsvThreshold" << std::endl;
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = hsvThresholdOutput;
	bool findContoursExternalOnly = false;  // default Boolean
	std::cout << "  Calling findContours" << std::endl;
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	//Step Filter_Contours0:
	//input
	std::vector<std::vector<cv::Point> > filterContoursContours = findContoursOutput;
	double filterContoursMinArea = 100.0;  // default Double
	double filterContoursMinPerimeter = 0.0;  // default Double
	double filterContoursMinWidth = 0.0;  // default Double
	double filterContoursMaxWidth = 40.0;  // default Double
	double filterContoursMinHeight = 18.0;  // default Double
	double filterContoursMaxHeight = 1000.0;  // default Double
	double filterContoursSolidity[] = {0.0, 100};
	double filterContoursMaxVertices = 1000000.0;  // default Double
	double filterContoursMinVertices = 0.0;  // default Double
	double filterContoursMinRatio = 0.0;  // default Double
	double filterContoursMaxRatio = 1000.0;  // default Double
	std::cout << "  Calling filterContours" << std::endl;
	filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->filterContoursOutput);
}

/**
 * This method is a generated setter for source0.
 * @param source the Mat to set
 */
void GripPipeline::setsource0(cv::Mat &source0){
	source0.copyTo(this->source0);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* GripPipeline::gethsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::getfindContoursOutput(){
	return &(this->findContoursOutput);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::getfilterContoursOutput(){
	return &(this->filterContoursOutput);
}
	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void GripPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		std::cout << "    hsvThreshold" << std::endl;
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		std::cout << "      cvtColor" << std::endl;
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
		std::cout << "      Scalar" << std::endl;
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void GripPipeline::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::cout << "    findContours" << std::endl;
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
		std::cout << "      findContours" << std::endl;
	}


	/**
	 * Filters through contours.
	 * @param inputContours is the input vector of contours.
	 * @param minArea is the minimum area of a contour that will be kept.
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
	 * @param minWidth minimum width of a contour.
	 * @param maxWidth maximum width.
	 * @param minHeight minimum height.
	 * @param maxHeight  maximimum height.
	 * @param solidity the minimum and maximum solidity of a contour.
	 * @param minVertexCount minimum vertex Count of the contours.
	 * @param maxVertexCount maximum vertex Count.
	 * @param minRatio minimum ratio of width to height.
	 * @param maxRatio maximum ratio of width to height.
	 * @param output vector of filtered contours.
	 */
	void GripPipeline::filterContours(std::vector<std::vector<cv::Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, std::vector<std::vector<cv::Point> > &output) {
		int i = 0;
		std::vector<cv::Point> hull;
		output.clear();
		std::cout << "Output cleared" << std::endl;
		for (std::vector<cv::Point> contour: inputContours) {
			std::cout << i++ << std::endl;
			cv::Rect bb = boundingRect(contour);
			std::cout << i++ << std::endl;
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			std::cout << i++ << std::endl;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			std::cout << i++ << std::endl;
			double area = cv::contourArea(contour);
			std::cout << i++ << std::endl;
			if (area < minArea) continue;
			std::cout << i++ << std::endl;
			if (arcLength(contour, true) < minPerimeter) continue;
			std::cout << i++ << std::endl;
			cv::convexHull(cv::Mat(contour, true), hull);
			std::cout << i++ << std::endl;
			double solid = 100 * area / cv::contourArea(hull);
			std::cout << i++ << std::endl;
			if (solid < solidity[0] || solid > solidity[1]) continue;
			std::cout << i++ << std::endl;
			if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
			std::cout << i++ << std::endl;
			double ratio = bb.width / bb.height;
			std::cout << i++ << std::endl;
			if (ratio < minRatio || ratio > maxRatio) continue;
			std::cout << i++ << std::endl;
			output.push_back(contour);
			std::cout << "Pushed back" << std::endl;
		}
	}



} // end grip namespace

