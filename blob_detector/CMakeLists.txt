cmake_minimum_required(VERSION 2.8)
project( blob_detector )
find_package( OpenCV REQUIRED )
add_executable( blob blobDetector.cpp )
target_link_libraries(blob cvblob)
target_link_libraries( blob ${OpenCV_LIBS} )
include_directories(${cvBlob_SOURCE_DIR}/cvBlob)


