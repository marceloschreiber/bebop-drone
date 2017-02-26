#barsa Version required cmake
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_C_COMPILER "/usr/bin/gcc")
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Specify cmake flags
SET(CMAKE_C_FLAGS "-g -Wall -std=c11")
set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

LINK_DIRECTORIES(
    /usr/local/lib
    /home/marcelo/IIT/drone/SDK/out/arsdk-native/staging/usr/lib
)
INCLUDE_DIRECTORIES(
    /usr/local/include
    /home/marcelo/IIT/drone/SDK/out/arsdk-native/staging/usr/include
)

#About this project
project( DronePiloting )

find_package( OpenCV REQUIRED )

file ( GLOB_RECURSE SRCS ./src/*.cpp ./src/*.hpp ./src/*.c ./src/*.h )

add_executable ( DronePiloting.out ${SRCS} )

TARGET_LINK_LIBRARIES( DronePiloting.out
	#-lopencv_calib3d
	#-lopencv_contrib
	#-lopencv_core
	#-lopencv_features2d
	#-lopencv_flann
	#-lopencv_gpu
	#-lopencv_highgui
	#-lopencv_imgproc
	#-lopencv_legacy
	#-lopencv_ml
	#-lopencv_nonfree
	#-lopencv_objdetect
	#-lopencv_ocl
	#-lopencv_photo
	#-lopencv_stitching
	#-lopencv_superres
	#-lopencv_ts
	#-lopencv_video
	#-lopencv_videostab
	-larcontroller
	-larsal
	-larcommands
	-larnetwork
	-larnetworkal
	-lardiscovery
	-larstream
	-lavcodec
	-lavformat
	-lswscale
	-lavutil
	-lncurses
    ${OpenCV_LIBS}
)
