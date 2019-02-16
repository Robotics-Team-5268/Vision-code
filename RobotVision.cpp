#include "RobotVision.hpp"

#include "Contour.h"
#include "GripPipeline.h"
#include "UDPSender.h"
#include <vector>

static void calculateContourAngle( Contour& aContour, std::vector<grip::Line>& filterLinesOutput );

void RobotVision::drawHWCA(cv::Mat &frame, std::vector<shape> &filterContoursOutput,std::vector<grip::Line> &filterLinesOutput, UDPSender *udpsender)
{ 
	std::vector<Contour> contours;
	
	for(shape shapes : filterContoursOutput)
	{
		Contour contour;
		contour.mBoundingBox = cv::boundingRect( shapes );
		contour.mCenterX = contour.mBoundingBox.x + contour.mBoundingBox.width / 2;
		contour.mCenterY = contour.mBoundingBox.y + contour.mBoundingBox.height / 2;
		calculateContourAngle( contour, filterLinesOutput );
		contours.push_back( contour );
	}

	udpsender->sendContours( contours );
	
}

static void calculateContourAngle
	(
	Contour& aContour, // Input/Output parameter
	std::vector<grip::Line>& filterLinesOutput
	)
{
	std::vector<grip::Line> contourLines;

	// Generate a slightly larger bounding box to incorporate lines on the edge
	cv::Rect inflatedRect( aContour.mBoundingBox );
	cv::Size inflationSize( 10, 10 );
	cv::Point offset( 5, 5 );
	inflatedRect += inflationSize;
	inflatedRect -= offset;
	std::cout << "bounding box ";
	std::cout << aContour.mBoundingBox.x << "-->" << inflatedRect.x << " " ;
	std::cout << aContour.mBoundingBox.y << "-->" << inflatedRect.y << " ";
	std::cout << aContour.mBoundingBox.width << "-->" << inflatedRect.width << " ";
	std::cout << aContour.mBoundingBox.height << "-->" << inflatedRect.height << std::endl;

	for( const grip::Line line : filterLinesOutput )
	{
		if( inflatedRect.contains( cv::Point( line.x1, line.y1 ) ) &&
			inflatedRect.contains( cv::Point( line.x2, line.y2 ) ) )
		{
			contourLines.push_back( line );
		}
		else
		{
			std::cout << "!!!" << "bx=" << inflatedRect.x;
			std::cout << "by=" << inflatedRect.y;
			std::cout << "bw=" << inflatedRect.width;
			std::cout << "bh=" << inflatedRect.height;
			std::cout << "x1=" << line.x1;
			std::cout << "y1=" << line.y1;
			std::cout << "x2=" << line.x2;
			std::cout << "y2=" << line.y2;
			std::cout << std::endl;
		}
	}

	float angle = 0;
	for( grip::Line line : contourLines )
	{
		if( line.angle() >= 0 )
		{
			angle += line.angle();
		}
		else
		{
			angle += line.angle() + 180.0f;
		}
		//std::cout << "ang=" << line.angle();
	}

	// Average the angles of the lines that represent the contour.  They *should* all be roughly equivalent
	// TODO: Should there be something more complex for outliers?
	if( contourLines.size() > 0 )
	{
	    aContour.mAngle = angle / contourLines.size();
	}

}
