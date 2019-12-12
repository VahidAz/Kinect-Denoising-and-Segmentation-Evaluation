/**
 * \addtogroup jcv_kinect_denoising
 *
 * namespace jcv_kinect_denoising
 *
 * @{
 */


#ifndef JCV_TEMPORAL_DENOISING__
#define JCV_TEMPORAL_DENOISING__


#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "opencv2/core/core.hpp"

#include <vector>


namespace jcv_kinect_denoising /** namespace jcv_kinect_denoising */
{
    /**
    * \class TemporalDenoise ""
    *
    * \brief spatial-temporal denoising by Bilateral Filter.
    *
    * This class is for denoising spatial-temporal noise in kinect frames.
    * In order to denoise, some frames before and after target frame plus some pixels in target frames are used.
    * Using frames in around is for temporal denoising and using from around pixels in target frame is for spatial denoising.
    *
    * \author Vahid Azizi
    *
    * \version 0.0.1
    *
    * \date 2015/04/25
    *
    * Contact: va.azizi@gmail.com
    *
    */
	class TemporalDenoise
	{
    	public:
        	TemporalDenoise();  /**< Constructor */
			~TemporalDenoise(); /**< Destructor  */
		
		private:
        	ros::NodeHandle nh;
        	ros::Subscriber sub;
			ros::Publisher  pub;

			std::vector<cv::Mat> bef; /**< Keeping frames in temporal radius */

        	int count,          /**< Checking temporal interval */
            	spatialRadius,  /**< Block size in frames */
            	temporalRadius, /**< Number of considered frames for denoising */
            	sigmaR,         /**< Spatial Gaussian for decreasing the influence of distant pixels */
            	sigmaS;         /**< Range Gaussian for decreasing the influence of high intensity pixels */

	    	double variance; /**< Depth variance of depth error */

			/**
			 * Spatial_Temporal DNBL
			 *
			 * @param _msg Recieved message from ROS, we need its information(header,...) for publishing result
			 * @param _type Type of published image
			 */
			inline void filter(const sensor_msgs::Image::ConstPtr&,int);
			
			void imageCallBack(const sensor_msgs::Image::ConstPtr&);
			
			/**
			 * 2D guassian kernel
			 *
			 * @param _x Input value for guassian kernel
			 * @param _sigma Sigma for guassian kernel
			 */
			inline const float gKernel  (const float&,const float&);
			
			/**
			 * Adaptive guassian kernel
			 *
			 * @param _x Input value for guassian kernel
			 * @param _sigma Sigma for guassian kernel
			 */
			inline const float aSGKernel(const float&,const float&);

	}; // class TemporalDenoise

}; // namespace jcv_kinect_denoising


#endif /* JCV_TEMPORAL_DENOISING__ */


/*! @} addtogroup jcv_kinect_denoising */
