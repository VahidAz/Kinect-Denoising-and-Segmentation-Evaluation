#include "opencv2/highgui/highgui.hpp"


void showInScreen(const std::string& _winName, const cv::Mat& _img, const int& _waitVal)
{
    double max;
    cv::Mat tmp;

    cv::minMaxLoc( _img, 0, &max, 0, 0 );
    _img.convertTo( tmp, CV_32F, 1.0/max, 0 );

    cv::namedWindow( _winName, cv::WINDOW_NORMAL );
    cv::imshow( _winName, tmp );
    cv::waitKey( _waitVal );
}
