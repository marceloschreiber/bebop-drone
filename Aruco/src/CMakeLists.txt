cmake_minimum_required(VERSION 2.8)
project(ARCode)
find_package( OpenCV REQUIRED )
add_executable( makeARCode makeARCode.cpp )
add_executable( readARCode readARCode.cpp )

target_link_libraries( makeARCode ${OpenCV_LIBS} )
target_link_libraries( readARCode ${OpenCV_LIBS} )
