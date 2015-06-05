cmake_minimum_required(VERSION 2.8)

project(polygonintersection)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(polygonintersection main.cpp)

target_link_libraries( polygonintersection ${OpenCV_LIBS} )

#install(TARGETS polygonintersection RUNTIME DESTINATION bin)
