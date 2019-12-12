cmake_minimum_required(VERSION 2.8.3)
project(jcv_temporal_denoising)

## Defualt build type is release, in order to have debug symbols ...
## please comment release build type and uncomment debug build type
set(CMAKE_BUILD_TYPE RELEASE)
#set(CMAKE_BUILD_TYPE DEBUG)

## DEBUG macro is for running program in debug mode ...
## in this mode each step is shown in screen
#add_definitions(-DDEBUG)
## FDEBUG macro is a setting for DEBUG macro that sets ...
## waitkey to 100 in cv::imshow
#add_definitions(-DFDEBUG)

find_package(catkin REQUIRED roscpp cv_bridge sensor_msgs)

find_package(OpenCV REQUIRED)

include_directories(include ${OPENCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${OPENCV_INCLUDE_DIRS}
  LIBRARIES  
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs
  DEPENDS OpenCV
)

add_executable(${PROJECT_NAME} src/main.cpp src/jcv_temporal_denoising.cpp src/jcv_debug.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(FILES launch/jcv_temporal_denoising.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Generating doxygen
#include(${CMAKE_CURRENT_SOURCE_DIR}/../jcv_kinect_denoising/DoxygenCMakeLists.txt)
