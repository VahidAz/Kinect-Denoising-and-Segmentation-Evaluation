#include "../include/jcv_temporal_denoising.hpp"


int main(int _argc, char** _argv)
{
    ros::init( _argc, _argv, "jcv_temporal_denoising" );
    
    jcv_kinect_denoising::TemporalDenoise temp_den;

    ros::spinOnce();

    try
    {
	ros::spin();
    }
    catch(std::runtime_error& _e)
    {
        ROS_ERROR( "runtime exeption: %s", _e.what() );

        return 1;
    }
    
    return 0;
}
