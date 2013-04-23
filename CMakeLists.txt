cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
link_directories(lib)
rosbuild_add_executable(test_blur src/test_blur.cpp)
rosbuild_add_executable(ComputeGradients src/ComputeGradients.cpp)
#rosbuild_add_executable(SobelOperator src/SobelOperator.cpp)
rosbuild_add_executable(ORBDetector src/ORBDetector.cpp)
rosbuild_add_executable(SIFTDetector src/SIFTDetector.cpp)
rosbuild_add_executable(BlurImage src/BlurImage.cpp)
rosbuild_add_executable(SimpleBlobDetector src/SimpleBlobDetector.cpp)
rosbuild_add_executable(QRCodeFinder src/QRCodeFinder.cpp)
rosbuild_add_executable(FaceDetector src/FaceDetector.cpp)
rosbuild_add_executable(PersonFollower src/PersonFollower)
<<<<<<< HEAD
rosbuild_add_executable(TestDirectory src/TestDirectory.cpp)
rosbuild_add_executable(FaceRecognizer src/FaceRecognizer.cpp)
=======
>>>>>>> 6f3e6e91ed599daecc349a26fd1f8d50b856a3ad
target_link_libraries(test_blur zxing)
target_link_libraries(QRCodeFinder zxing)
