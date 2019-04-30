#[[cmake_minimum_required(VERSION 3.13)
project(untitled3)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
find_path(OPEN_NI_INCLUDE /usr/lib/OpenNI2)

add_executable(untitled3 main.cpp Viewer.cpp)

INCLUDE_DIRECTORIES(/home/hoit/Downloads/OpenNI_2.3.0.55/Linux/OpenNI-Linux-x64-2.3.0.55/Include)
INCLUDE_DIRECTORIES(/home/hoit/CLionProjects/untitled3)
#INCLUDE_DIRECTORIES(/home/hoit/CLionProjects/untitled3/Redist/OpenNI2)

link_directories(/home/hoit/CLionProjects/untitled3)
#link_directories(/home/hoit/CLionProjects/untitled3/Redist/OpenNI2)
link_directories(/home/hoit/Downloads/OpenNI_2.3.0.55/Linux/OpenNI-Linux-x64-2.3.0.55/Include)

#link_libraries(libOpenNI2.so libOpenNI2.jni.so libMWClosestPoint.so libOniFile.so liborbbec.so libPSLink.so)
link_libraries(libOpenNI2.so libOpenNI2.jni.so)


#[=[

FIND_PATH(OPEN_NI_INCLUDE "XnOpenNI.h" "OpenNIConfig.h" HINTS "$ENV{OPEN_NI_INCLUDE}" "/usr/include/ni" "/usr/include/openni" "/opt/ros/groovy/include/openni_camera")
FIND_LIBRARY(OPEN_NI_LIBRARY NAMES OpenNI libOpenNI HINTS $ENV{OPEN_NI_LIB} "/usr/lib")
LINK_DIRECTORIES($ENV{OPEN_NI_LIB})
INCLUDE_DIRECTORIES(${OPEN_NI_INCLUDE})
LINK_LIBRARIES(${OPEN_NI_LIBRARY})
FIND_LIBRARY(OPEN_NI_LIBRARY NAMES OpenNI libOpenNI HINTS $ENV{OPEN_NI_LIB} "/usr/lib")
target_link_libraries (untitled3 ${SSE_FLAGS} ${OPEN_NI_LIBRARIES} ${XN_NITE_LIBRARIES})
]=]

target_link_libraries(untitled3 OpenGL)
target_link_libraries(untitled3 glut)
target_link_libraries(untitled3 OpenNI2)
target_link_libraries(untitled3 ${OpenCV_LIBS})]]




cmake_minimum_required(VERSION 3.13)
project(untitled3)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)

include_directories("/usr/local/include/apriltag")
INCLUDE_DIRECTORIES(/home/hoit/Downloads/OpenNI_2.3.0.55/Linux/OpenNI-Linux-x64-2.3.0.55/Include) #header
#LINK_LIBRARIES( OpenNI2 ) #lib name
link_directories(/home/hoit/Downloads/OpenNI_2.3.0.55/Linux/OpenNI-Linux-x64-2.3.0.55/Redist)   #dir. of lib.    include all below

add_executable(untitled3 main.cpp Viewer.cpp GoCart_vision.cpp GoCart_vision.h) # need to include all .cpp

target_link_libraries(untitled3 OpenGL) #3rd party lib
target_link_libraries(untitled3 glut)   #3rd party lib
target_link_libraries(untitled3 OpenNI2) #3rd party lib(normally this only, but in this case orbbec driver is needed. That is why include another dir. in /Downloads)
target_link_libraries(untitled3 ${OpenCV_LIBS})#3rd party lib
target_link_libraries(untitled3 apriltag)

