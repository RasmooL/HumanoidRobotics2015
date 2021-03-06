cmake_minimum_required(VERSION 2.8.3)
project(imitation)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_system")

set(CMAKE_MODULE_PATH #${CMAKE_MODULE_PATH} 
	/usr/local/lib/cmake 
	/opt/ros/indigo/share/cmake_modules/cmake/Modules 
	${PROJECT_SOURCE_DIR}/cmake
)



find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  cv_bridge
  image_transport
)

catkin_package()
#find_package(Naoqi REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS} #${NAOQI_INCLUDE_DIR}
)

add_executable(	imitation src/main.cpp 
				src/Kinematics/NAOKinematics.cpp 
				src/Kinematics/NAOKinematicsJacobian.cpp
		
				src/srcVision/arm.cpp
				src/srcVision/aruco_vision.cpp
				src/srcVision/joint.cpp

				src/control_adam.cpp
				src/control_rasmus.cpp
				src/control_florian.cpp
				src/optim.cpp
				src/load.cpp
				)


#target_link_libraries(imitation ${NAOQI_LIBRARIES})
target_link_libraries(imitation ${catkin_LIBRARIES})

find_package(Eigen REQUIRED)
target_link_libraries(imitation ${Eigen_LIBRARIES})

#find_package(Boost REQUIRED COMPONENTS system thread)
#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(imitation ${Boost_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(imitation ${OpenCV_LIBS})
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
find_package(aruco REQUIRED)
target_link_libraries(imitation  ${aruco_LIBS})
