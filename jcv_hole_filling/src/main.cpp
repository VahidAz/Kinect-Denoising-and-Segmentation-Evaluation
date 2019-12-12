#include "../include/jcv_hole_filling.hpp"


int main(int _argc, char** _argv)
{
	ros::init( _argc, _argv, "jcv_hole_filling" );
    
	jcv_kinect_denoising::HoleFilling hole_filling;

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
