/**
 * \addtogroup jcv_kinect_denoising
 *
 * namespace jcv_kinect_denoising
 *
 * @{
 */


#ifndef JCV_HOLE_FILLING_HPP__
#define JCV_HOLE_FILLING_HPP__


#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "opencv2/core/core.hpp"


namespace jcv_kinect_denoising /** namespace jcv_kinect_denoising */
{
    /**
    * \class HoleFilling ""
    *
    * \brief Filling invalid pixels (0 or NAN) in kinect frames and refining them(denoising spatial noise).
    *
    * This class is for filling invalid pixels in kinect frames and refining them. 
    * The value of valid pixels around invalid pixels are used for estimating new value. 
    * And for refining the Bilateral Filter is used.
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
    class HoleFilling
    {
        public:
            HoleFilling();  /**< Constructor */
            ~HoleFilling(); /**< Destructor  */
        
        private:
            ros::NodeHandle nh;
            ros::Subscriber sub;
            ros::Publisher  pub;
            
            cv::Mat ptrTmp;    // Keeping current frame for increasing speed
            
            bool nanFlag;      /**< Indicating that invalid pixels are 0 or NAN, true means they are NAN and false means 0 */

            int igThre,        /**< Threshold for ignoring contours with big size, usualy around frames */
                spatialRadius, /**< The size of window for applying bilateral filter */
                sigmaS,        /**< Spatial Gaussian for decreasing the influence of distant pixels */
                sigmaR;	       /**< Range Gaussian for decreasing the influence of high intensity pixels */
            
            void imageCallBack(const sensor_msgs::Image::ConstPtr&);
            
            /**
             * Converting depth image to binary and applying dilatation
             *
             * @param _dil_bin Output(The binary dilated depth image)
             */
            inline void binaryDilate(cv::Mat&);
            
            /**
             * Filling invalid pixels with new value(main process)
             *
             * @param _dil_bin Filled frame
             */ 
            inline void paddingHoles(cv::Mat&);
            
            /**
             * Converting depth image with NAN value for invalid pixels to binary depth image
             * For depth image that 0 indicates invalid pixels the cv::threshold is used.
             *
             * @param _src Input image
             * @param _bin Output(Binary Image)
             */
            inline void thresholdNAN(const cv::Mat&,cv::Mat&);

    }; // class HoleFilling

}; // namespace jcv_kinect_denoising


#endif /* JCV_HOLE_FILLING_HPP__ */


/*! @} addtogroup jcv_kinect_denoising */
