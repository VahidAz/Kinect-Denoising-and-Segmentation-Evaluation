#include "../include/jcv_debug.hpp"
#include "../include/jcv_temporal_denoising.hpp"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


jcv_kinect_denoising::TemporalDenoise::TemporalDenoise(): count(-1), variance(2), spatialRadius(3), 
                                                          temporalRadius(3), sigmaR(2), sigmaS(2)
{
	ros::NodeHandle private_nh = ros::NodeHandle( "~" );

	private_nh.param( "variance"      , this->variance      , this->variance       );
	private_nh.param( "spatialRadius" , this->spatialRadius , this->spatialRadius  );
	private_nh.param( "temporalRadius", this->temporalRadius, this->temporalRadius );
	private_nh.param( "sigmaR"        , this->sigmaR        , this->sigmaR         );
	private_nh.param( "sigmaS"        , this->sigmaS        , this->sigmaS         );

   // Subscribing to result of jcv_hole_filling
   this->sub = nh.subscribe( "/jcv_hole_filling/image", 1, &TemporalDenoise::imageCallBack, this );

   this->pub = nh.advertise<sensor_msgs::Image>( "/jcv_temporal_denoising/image", 1 );
}


jcv_kinect_denoising::TemporalDenoise::~TemporalDenoise()
{
}	


void jcv_kinect_denoising::TemporalDenoise::imageCallBack(const sensor_msgs::Image::ConstPtr& _msg)
{
    // Converting from sensor_msg format to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare( _msg );
    }
    catch (const cv_bridge::Exception& _e)
    {
        ROS_ERROR( "cv_bridge exception: %s", _e.what() );
    }

    ++this->count; // Counting to check temporal interval
    
    // Extending bounderies for convolution, filling with constant value (0)
    cv::Mat bound_frame, result;
    int extra_bound = this->spatialRadius / 2;

    cv::copyMakeBorder( cv_ptr->image, bound_frame, extra_bound, extra_bound,
                        extra_bound, extra_bound, cv::BORDER_CONSTANT, cv::Scalar(0) );

    bound_frame.convertTo( bound_frame, CV_16U );

    if ( this->count == this->temporalRadius - 1 )
    {
	    this->bef.push_back( bound_frame.clone() );
	    this->count = -1; // Resetting count
        this->filter( _msg, cv_ptr->image.type() );	// Calling spatial-temporal denoising
	    this->bef.clear();
    }
    else
	    this->bef.push_back( bound_frame.clone() );
}


// Main method for denoising spatial-temporal noise
inline void jcv_kinect_denoising::TemporalDenoise::filter(const sensor_msgs::Image::ConstPtr& _msg, int _type)
{
    // Current frame for denoising
    cv::Mat cur = this->bef.at( this->temporalRadius/2 ).clone();

    LOG( "Cur Size:", cur.size() );
    SHOWINSCREEN( "Original", cur );

    // Tracing all pixels in frame
    // Accesing line by line, cuz it is more safe and fast
    ushort *rows_pixes, *roi_pixes;

    cv::Point start_coords;

    std::vector<cv::Mat> roi;

    int mid_block = this->spatialRadius / 2;

    ushort IP; // Intensity of target pixel for denoising

    float sum_all,           // Second sigma in formula
          wp_all,            // WP in formula
          sum_each, wp_each, // Temporary variables for each block
          intensity_diff, spatial_diff, res_tmp, wp_tmp;

    cv::Mat final_res( cur.rows - 2 * mid_block, cur.cols - 2 * mid_block, CV_32F, cv::Scalar(0) );

    LOG( "Final Size:", final_res.size() );

    for ( int i = mid_block ; i < cur.rows - mid_block ; ++i )
    {
        rows_pixes = cur.ptr<ushort>(i);

        // Start coordination(x) for extracting block in size of spatialRadius * spatialRadius
        start_coords.x = i - mid_block;

        for ( int j = mid_block ; j < cur.cols - mid_block ; ++j )
        {
            if ( rows_pixes[j] != rows_pixes[j] || rows_pixes[j] == 0 ) // Igonoring NAN or 0
                continue;

           // Start coordination(y) for extracting block in size of spatialRadius * spatialRadius
           start_coords.y = j - mid_block;

           // Extracting corresponded blocks from frames in temporal radius and current frame
		   roi.clear();
           for ( int k = 0 ; k < this->temporalRadius ; ++k )
                roi.push_back( this->bef.at(k)( cv::Rect( start_coords.y, start_coords.x, this->spatialRadius, this->spatialRadius ) ) );

           IP = roi.at( this->temporalRadius/2 ).at<ushort>( mid_block, mid_block );
		   
		   LOG ( "IP(current value):", IP );

           sum_all = 0.0;     // Second sigma in formula
           wp_all  = 0.0;     // WP in formula

           // First sigma in formula
           for ( int k = 0 ; k < this->temporalRadius ; ++k )
           {
               // Resetting temp variables
               sum_each = 0.0;
               wp_each  = 0.0;

               for ( int m = 0 ; m < roi.at(k).rows ; ++m )
               {
                   roi_pixes = roi.at(k).ptr<ushort>(m);

                   for ( int n = 0 ; n < roi.at(k).cols ; ++n )
                   {
                       if ( k == this->temporalRadius/2 && mid_block == m && mid_block == n ) // Ignoring the goal pixel for denoising
                           continue;

                       if ( roi_pixes[n] == 0 || roi_pixes[n] != roi_pixes[n] ) // Ignoring NAN and 0
                           continue;

                       intensity_diff = abs( IP - roi_pixes[n] );
                       spatial_diff = sqrt( ( ( mid_block - m ) * ( mid_block - m ) ) + ( ( mid_block -n ) * ( mid_block - n ) ) );
                       res_tmp = gKernel( spatial_diff, this->sigmaS ) * aSGKernel( intensity_diff, this->sigmaR ) * roi_pixes[n];
                       wp_tmp = gKernel( spatial_diff, this->sigmaR ) * aSGKernel( intensity_diff, this->sigmaR );

                       wp_each  += wp_tmp;
                       sum_each += res_tmp;
                   }
               }

               sum_all += sum_each;
               wp_all += wp_each;
           }
		   
		   LOG( "New value:", sum_all/wp_all );
		   
		   final_res.at<float>(i,j) = sum_all/wp_all;
		   
		   SHOWINSCREEN( "Result", final_res );
		}
	}

    // Publishing the result 
    final_res.convertTo( final_res, _type );
    cv_bridge::CvImage out_msg;

    out_msg.header   = _msg->header;
    out_msg.encoding = _msg.get()->encoding;
    out_msg.image    = final_res; 
    
    sensor_msgs::Image ros_image;
    out_msg.toImageMsg( ros_image );   
    
    this->pub.publish( ros_image  );

    SHOWINSCREEN( "Result", final_res );

    #ifdef DEBUG
        // Closeing all windows    
        cv::destroyAllWindows();
    #endif
}


// 2D Gaussian Kernel
inline const float jcv_kinect_denoising::TemporalDenoise::gKernel(const float& _x, const float& _sigma)
{
    return 1 / ( 2 * M_PI * _sigma * _sigma ) * exp( ( -_x * _x ) / ( 2 * _sigma * _sigma ) );
}


// Adaptive Scaling Gaussian Kernel
inline const float jcv_kinect_denoising::TemporalDenoise::aSGKernel(const float& _x, const float& _sigma)
{
    return 1 / ( 2 * M_PI * _sigma * _sigma ) * exp( ( -_x * _x ) / ( 2 * _sigma * _sigma * this->variance ) );
}
