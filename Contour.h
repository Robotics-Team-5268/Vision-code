#pragma once

#include <opencv2/core/core.hpp>

class Contour
{
public:
    Contour()
        : mAngle( 0 )
        , mCenterX( 0 )
        , mCenterY( 0 )
        {

        }

    int mAngle;
    int mCenterX;
    int mCenterY;
    cv::Rect mBoundingBox;
};


