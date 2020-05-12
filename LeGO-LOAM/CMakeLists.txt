cmake_minimum_required(VERSION 2.8.3)
project(lego_loam_bor)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3") # From LeGO-LOAM
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g ") # From LeGO-LOAM-BOR
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -g") # 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -g ") # robot54: works

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge # robot54: needed for Scan Context loop detector.
  image_transport
  rosbag_storage

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE) # robot54: needed for Scan Context loop detector.
find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET) # robot54: needed for Scan Context loop detector.

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs rosbag_storage
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}	# robot54: needed for Scan Context loop detector.  
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS} # robot54: needed for Scan Context loop detector.
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)


add_executable(sc_lego_loam_bor
    src/imageProjection.cpp
    src/featureAssociation.cpp
    src/mapOptmization.cpp
    src/transformFusion.cpp
    src/main.cpp
    src/Scancontext.cpp # robot54: needed for Scan Context loop detector.
)

add_dependencies(sc_lego_loam_bor ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(sc_lego_loam_bor ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Eigen3::Eigen)
	# robot54: 'Eigen3::Eigen' needed for Scan Context loop detector.
