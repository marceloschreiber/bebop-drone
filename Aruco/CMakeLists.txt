cmake_minimum_required(VERSION 2.8)
project(ARCode)
find_package( OpenCV REQUIRED )
add_executable( ARCode src/main.cpp )
target_link_libraries( ARCode ${OpenCV_LIBS} )
