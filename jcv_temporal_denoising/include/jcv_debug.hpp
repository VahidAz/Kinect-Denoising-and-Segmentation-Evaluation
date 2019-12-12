#ifndef JCV_DEBUG_HPP__
#define JCV_DEBUG_HPP__


#include "opencv2/core/core.hpp"


void showInScreen(const std::string&,const cv::Mat&,const int&);


#if FDEBUG
	#define DEFWAIT 100
#else
	#define DEFWAIT 0
#endif // FDEBUG


#ifdef DEBUG
	
	#define LOG2( _msg, _args ) std::cerr << _msg << "   " <<  _args << std::endl

	#define LOG1( _msg ) std::cerr << _msg << std::endl

	#define LOGCALLBYNUM( _msg, _0, N, ... ) LOG##N

	#define LOG( _msg, _args... ) LOGCALLBYNUM( _msg, ## _args, 2, 1 )( _msg, ## _args )

			
	#define SHOWINSCREEN2( _winName, _img ) showInScreen( _winName, _img, DEFWAIT )

	#define SHOWINSCREEN3( _winName, _img, _waitVal ) showInScreen( _winName, _img, _waitVal )

	#define SHOWCALLBYNUM( _0, _1, N, ... ) SHOWINSCREEN##N

	#define SHOWINSCREEN( _winName, _img, _args... ) \
	        SHOWCALLBYNUM( _img, ## _args, 3, 2 )( _winName, _img, ## _args )

#else

	#define LOG( _msg, _args... )                    ((void)0)
	#define SHOWINSCREEN( _winName, _img, _args... ) ((void)0)

#endif // DEBUG MACRO


#endif /* JCV_DEBUG_HPP__ */
