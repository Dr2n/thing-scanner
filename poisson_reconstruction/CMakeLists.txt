CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# localize the PCLConfig.cmake file in the following line!
SET(PCL_DIR "/usr/local/Cellar/pcl/1.9.1_4/cmake/PCLConfig.cmake")

PROJECT(ar)

#complier
SET( CMAKE_CXX_COMPILER "g++")

#executable
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

#library
#SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(/usr/local/lib)
#LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

#header file
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)


#pcl library
FIND_PACKAGE( PCL REQUIRED )
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS})
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS})

SET(SOURCES src/main.cpp src/load_data.cpp src/poisson.cpp)

add_executable(main ${SOURCES})
TARGET_LINK_LIBRARIES(main ${OpenCV_LIBS} ${PCL_LIBRARIES})


