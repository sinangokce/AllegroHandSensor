cmake_minimum_required(VERSION 2.8.3)
project(glove_tekscan_ros_wrapper)


find_package(catkin REQUIRED COMPONENTS geometry_msgs rosbag roscpp tf message_generation cmake_modules )
FIND_PACKAGE(YARP)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
   FILES
   HandWritingDataWrapper.msg
   LasaDataStreamWrapper.msg
 )

generate_messages(
  DEPENDENCIES  std_msgs geometry_msgs sensor_msgs # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include  
#  LIBRARIES glove_tekscan_ros_wrapper
  CATKIN_DEPENDS message_runtime geometry_msgs rosbag roscpp tf 
 # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(IntTacMain src/IntTacMain.cpp)
target_link_libraries(IntTacMain  ${catkin_LIBRARIES} ${YARP_LIBRARIES})
add_dependencies(IntTacMain glove_tekscan_ros_wrapper_generate_messages_cpp)
