# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# compile CXX with g++
CXX_FLAGS =       -fPIC -std=gnu++11

CXX_DEFINES = -DDISABLE_DAVIDSDK -DDISABLE_DSSDK -DDISABLE_ENSENSO -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DDISABLE_RSSDK -DFLANN_STATIC -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG -DQT_WIDGETS_LIB -Dqh_QHpointer -DvtkFiltersFlowPaths_AUTOINIT="1(vtkFiltersParallelFlowPaths)" -DvtkIOExodus_AUTOINIT="1(vtkIOParallelExodus)" -DvtkIOGeometry_AUTOINIT="1(vtkIOMPIParallel)" -DvtkIOImage_AUTOINIT="1(vtkIOMPIImage)" -DvtkIOParallel_AUTOINIT="1(vtkIOMPIParallel)" -DvtkIOSQL_AUTOINIT="2(vtkIOMySQL,vtkIOPostgreSQL)" -DvtkRenderingContext2D_AUTOINIT="1(vtkRenderingContextOpenGL)" -DvtkRenderingCore_AUTOINIT="3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL)" -DvtkRenderingFreeType_AUTOINIT="2(vtkRenderingFreeTypeFontConfig,vtkRenderingMatplotlib)" -DvtkRenderingLIC_AUTOINIT="1(vtkRenderingParallelLIC)" -DvtkRenderingVolume_AUTOINIT="1(vtkRenderingVolumeOpenGL)"

CXX_INCLUDES = -I"/home/xiaoyi/ar&3d project/include" -I/usr/include/vtk-6.3 -I/usr/include/freetype2 -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent/include -I/usr/lib/x86_64-linux-gnu/openmpi/include -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu -I/usr/include/hdf5/openmpi -I/usr/include/libxml2 -I/usr/include/jsoncpp -I/usr/include/tcl -I/usr/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/include/ni -I/usr/include/openni2 -isystem /home/xiaoyi/opencv/opencv/build -isystem /home/xiaoyi/opencv/opencv/include -isystem /home/xiaoyi/opencv/opencv/modules/core/include -isystem /home/xiaoyi/opencv/opencv/modules/flann/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/hdf/include -isystem /home/xiaoyi/opencv/opencv/modules/imgproc/include -isystem /home/xiaoyi/opencv/opencv/modules/ml/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/phase_unwrapping/include -isystem /home/xiaoyi/opencv/opencv/modules/photo/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/plot/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/quality/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/reg/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/surface_matching/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/viz/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/xphoto/include -isystem /home/xiaoyi/opencv/opencv/modules/dnn/include -isystem /home/xiaoyi/opencv/opencv/modules/features2d/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/freetype/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/fuzzy/include -isystem /home/xiaoyi/opencv/opencv/modules/gapi/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/hfs/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/img_hash/include -isystem /home/xiaoyi/opencv/opencv/modules/imgcodecs/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/line_descriptor/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/saliency/include -isystem /home/xiaoyi/opencv/opencv/modules/videoio/include -isystem /home/xiaoyi/opencv/opencv/modules/calib3d/include -isystem /home/xiaoyi/opencv/opencv/modules/highgui/include -isystem /home/xiaoyi/opencv/opencv/modules/objdetect/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/rgbd/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/shape/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/structured_light/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/text/include -isystem /home/xiaoyi/opencv/opencv/modules/ts/include -isystem /home/xiaoyi/opencv/opencv/modules/video/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/videostab/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/xfeatures2d/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/ximgproc/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/xobjdetect/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/aruco/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/bgsegm/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/bioinspired/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/ccalib/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/datasets/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/dnn_objdetect/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/dpm/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/face/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/optflow/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/sfm/include -isystem /home/xiaoyi/opencv/opencv/modules/stitching/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/superres/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/tracking/include -isystem /home/xiaoyi/opencv/opencv_contrib/modules/stereo/include -isystem /usr/include/x86_64-linux-gnu/qt5 -isystem /usr/include/x86_64-linux-gnu/qt5/QtWidgets -isystem /usr/include/x86_64-linux-gnu/qt5/QtGui -isystem /usr/include/x86_64-linux-gnu/qt5/QtCore -isystem /usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++ 
