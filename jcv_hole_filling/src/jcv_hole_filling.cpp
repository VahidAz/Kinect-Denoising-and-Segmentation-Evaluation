#include "../include/jcv_debug.hpp"
#include "../include/jcv_hole_filling.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <cassert>


jcv_kinect_denoising::HoleFilling::HoleFilling(): nanFlag(false), spatialRadius(5), sigmaS(20), sigmaR(20)
{
	ros::NodeHandle private_nh = ros::NodeHandle( "~" );

	private_nh.param( "nanFlag"      , this->nanFlag      , this->nanFlag       );
	private_nh.param( "spatialRadius", this->spatialRadius, this->spatialRadius );
	private_nh.param( "sigmaR"       , this->sigmaR       , this->sigmaR        );
	private_nh.param( "sigmaS"       , this->sigmaS       , this->sigmaS        );

	if ( nanFlag ) // If invalid pixels are marked with NAN
	{
		// Subscribe to standard kinect topics ( NAN stands for holes )
		this->sub = this->nh.subscribe( "/camera/depth/image", 1, &HoleFilling::imageCallBack, this );
		this->igThre = 2000;
	}
	else // If invalid pixels are marked with 0
	{
		// Subscribe to bag files of teddies ( 0 stands for holes )
		this->sub = this->nh.subscribe( "/camera1/depth_registered/image_raw", 1, &HoleFilling::imageCallBack, this );
		this->igThre = 1500;
	}

	private_nh.param( "igThre", this->igThre, this->igThre );

	this->pub = this->nh.advertise<sensor_msgs::Image>( "/jcv_hole_filling/image", 1 );
}


jcv_kinect_denoising::HoleFilling::~HoleFilling()
{
}


void jcv_kinect_denoising::HoleFilling::imageCallBack(const sensor_msgs::Image::ConstPtr& _msg)
{
    LOG("++++++++++++++++++++ NEW FRAME ++++++++++++++++++++");

    // Converting from sensor_msg to cv::Mat
    // cv_ptr is read-only
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare( _msg );
    }
    catch (const cv_bridge::Exception& _e)
    {
        ROS_ERROR( "cv_bridge exception: %s", _e.what() );
    }

    // Creating a deep copy of current frame for having access to it in whole program
    cv_ptr->image.copyTo( this->ptrTmp );

    // This convert is because of threshold function
    // Threshold function supports CV_8U or CV_32F
    ptrTmp.convertTo( this->ptrTmp, CV_32F );

    SHOWINSCREEN( "Original", this->ptrTmp );

    cv::Mat dil_bin;

    // Converting to binary image and applying dilatation
    this->binaryDilate( dil_bin );

    // Filling invalid pixels, main proccess
    this->paddingHoles( dil_bin );

    SHOWINSCREEN( "Padding holes result", this->ptrTmp );

    // Refining filled depth image with bilateral filter(denoising spatial noise)
    cv::bilateralFilter( this->ptrTmp, dil_bin, this->spatialRadius, this->sigmaR, this->sigmaS ); 

    SHOWINSCREEN( "BF", dil_bin );

    // Publishing result 
    dil_bin.convertTo( dil_bin, cv_ptr->image.type() );

    cv_bridge::CvImage out_msg;

    out_msg.header   = _msg->header;
    out_msg.encoding = _msg.get()->encoding;
    out_msg.image    = dil_bin; 
    
    sensor_msgs::Image ros_image;
    out_msg.toImageMsg( ros_image );   
    
    this->pub.publish( ros_image  );

    #ifdef DEBUG
        // Closing all windows    
        cv::destroyAllWindows();
    #endif
}


// Converting to binary and dilating
inline void jcv_kinect_denoising::HoleFilling::binaryDilate(cv::Mat& _dil_bin)
{
    cv::Mat binary;
    
    // Converting to binary image
    if ( nanFlag ) // If NAN indicates invalid pixels
		this->thresholdNAN( this->ptrTmp, binary );
    else // If 0 indicates invalid pixels
    {
		cv::threshold( this->ptrTmp, binary, 0, 255, cv::THRESH_BINARY_INV );

		// In cv::threshold function the result will be in type of source image (CV_32F)
		// But we need to convert it to CV_8U
		binary.convertTo( binary, CV_8U );
    }

    SHOWINSCREEN( "Binary", binary );

    // Dilating image
    // Dilating with more iteration because pixels in edges are noisy
    // It is a tradeoff, big iteration causes to consider far pixels which are irrespective to considered hole
    cv::dilate( binary, _dil_bin, cv::Mat(), cv::Point(-1,-1), 1 ); 

    SHOWINSCREEN( "Dilate", _dil_bin );
}


// Finding holes and assigning new value to them
inline void jcv_kinect_denoising::HoleFilling::paddingHoles(cv::Mat& _dil_bin)
{
    // Applying contour
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    findContours( _dil_bin, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    LOG ( "Contour Size:", contours.size() );

    // Processing
    cv::Mat cont_ar( _dil_bin.rows, _dil_bin.cols, CV_8U );
    cv::Mat cont_in( _dil_bin.rows, _dil_bin.cols, CV_8U );

    cv::Mat inside, mask, hist, coor_inside;
    double max;
    int histSize, nonZIn, count, thre;
    float range[2], sum, percent, avg, binVal, old_val;
    const float* histRange;
    cv::Point p;

    // Processing all contours
    for ( int i = 0 ; i < contours.size() ; ++i )
    {
	    LOG( "-----------------------------------------" );
	    LOG( "Contour No.:", i );
	    LOG( "Size:", contours[i].size() );

	    // It is set because of ignoring high level contour in hierarchy
	    if ( contours[i].size() >= this->igThre )
	    {
		    LOG( "Ignoring because of size." );
		    continue;
	    }

	    cont_ar.setTo( cv::Scalar(0) ); 
	    cont_in.setTo( cv::Scalar(0) ); 
	    mask.setTo   ( cv::Scalar(0) ); 

	    cv::drawContours( cont_ar, contours, i, 255, 1, 8, cv::Mat(), 0 ); // Around pixels
	    LOG( "Number of Pixs Around:", cv::countNonZero(cont_ar) );
	    SHOWINSCREEN( "Contour_Around", cont_ar );

	    cv::drawContours( cont_in, contours, i, 255, CV_FILLED, 8, cv::Mat(), 0 ); // All pixels(around+inside)
	    LOG( "Number of Pixs Around + Inside:", cv::countNonZero(cont_in) );
	    SHOWINSCREEN( "Contour_Around_Inside", cont_in );

	    inside = cont_in - cont_ar; // Inside pixels
	    LOG( "Number of Pixs Inside:", cv::countNonZero(inside) );
	    SHOWINSCREEN( "Contour_Inside", inside );

	    // Selecting considered region in main image
	    this->ptrTmp.copyTo( mask, cont_ar );
	    SHOWINSCREEN( "Mask", mask );
	    
	    cv::minMaxLoc( mask, 0, &max, 0, 0 );
	    LOG( "max:", max );

	    histSize = max + 1;
	    range[0] = 0;
	    range[1] = float(histSize);
	    histRange = range;
	    
	    // Calculating histogram 
	    calcHist( &mask, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false );
 	    LOG( "Hist Size:", hist.size() );
 	    LOG( "Hist", hist );

	    percent = 0.1;
	    
	    // counting the number of nonzero pixels inside the holes 
	    nonZIn = cv::countNonZero( inside );

	    // For assuring to select bins with dominant peaks, we start with 10% and decrease it gradualy(dividing by 2)
	    hist.at<float>(0,0) = 0; // Assigning 0 to first bin, because image is full of zero

	    for(;;)
	    {
	    	thre = percent * nonZIn;
			LOG( "percent:", percent );
	    	LOG( "nonZero:", nonZIn  );
	    	LOG( "Thre:"   , thre    );
			
			cv::minMaxLoc( hist, 0, &max, 0, 0 );
			LOG( "max:", max );
			
			if ( max > thre )
					break;
			
			percent /= 2.0;
	    }
 		
	    sum = 0.0;
	    count = 0;

	    for( int h = 1 ; h < hist.total() ; ++h )
	    {
		    binVal = hist.at<float>(h);

		    if ( binVal > thre )
		    {
			    LOG( "BinVal:", binVal );
			    LOG( "h:", h );
				sum += (h * binVal);
				count += binVal;
		    }
	    }
	    
	    LOG( "SUM:", sum );
	    LOG( "Count:", count );
	    avg = sum/count;
	    LOG( "Avg:", avg );
	    
	    // Finding nonZero pixels for filling with new value
	    cv::findNonZero( inside, coor_inside ); 
	    for ( int k = 0 ; k < coor_inside.total() ; ++k )
	    {
		    p = coor_inside.at<cv::Point>(k);
		    LOG( "P:", p );
		    old_val = ptrTmp.at<float>(p.y,p.x);
		    LOG( "Old:", old_val );
		    
		    if ( old_val > 0 )
			    continue;

		    ptrTmp.at<float>(p.y,p.x) = avg;
	    }

	    SHOWINSCREEN( "Result Step by Step", this->ptrTmp );
    }
}


// Converting to binary in the case that NAN is indicating value for invalid pixels
inline void jcv_kinect_denoising::HoleFilling::thresholdNAN(const cv::Mat& _src, cv::Mat& _bin)
{
	cv::Mat binary( this->ptrTmp.rows, this->ptrTmp.cols, CV_8U, cv::Scalar(0) );

	assert( _src.type() == CV_32F );

	const float*   src_rows_pixes;
	unsigned char* bin_rows_pixes;

	for ( int i = 0 ; i < _src.rows ; ++i )
	{
		src_rows_pixes = _src.ptr<float>(i);
		bin_rows_pixes = binary.ptr<unsigned char>(i);

		for ( int j = 0 ; j < _src.cols ; ++j )
			if ( src_rows_pixes[j] != src_rows_pixes[j] ) // NAN
				bin_rows_pixes[j] = 255;
	}

	binary.copyTo( _bin );
}
