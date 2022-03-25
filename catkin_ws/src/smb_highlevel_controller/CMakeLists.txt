cmake_minimum_required(VERSION 3.0.2)
project(smb_highlevel_controller)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)
catkin_package(
        INCLUDE_DIRS
        include
        #LIBRARIES smb_highlevel_controller
        CATKIN_DEPENDS roscpp sensor_msgs
        #DEPENDS system_lib_LIBRARIES
)

include_directories(

  include ${catkin_INCLUDE_DIRS}
)


add_executable( scan_suscriber src/scan_suscriber.cpp)
target_link_libraries(scan_suscriber ${catkin_LIBRARIES})

add_executable(cmd_velpublisher src/cmd_vel_publisher.cpp)
target_link_libraries(cmd_velpublisher ${catkin_LIBRARIES})

