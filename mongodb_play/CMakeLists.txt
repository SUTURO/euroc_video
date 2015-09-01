cmake_minimum_required(VERSION 2.8.3)
project(mongodb_play)

find_package(catkin REQUIRED roslib rostopic tf std_msgs sensor_msgs suturo_video_msgs)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES db_player string_player tf_player image_player jointstate_player gazebo_to_tf_player
  CATKIN_DEPENDS roscpp std_msgs suturo_video_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LINK_DIRS})

add_library(db_player src/db_player.cpp )
add_library(string_player src/string_player.cpp )
add_library(tf_player src/tf_player.cpp )
add_library(image_player src/image_player.cpp )
add_library(jointstate_player src/jointstate_player.cpp )
add_library(gazebo_to_tf_player src/gazebo_to_tf_player.cpp )

## Declare a cpp executable
add_executable(mongodb_play src/mongodb_play.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(mongodb_play db_player string_player tf_player image_player gazebo_to_tf_player jointstate_player suturo_video_msgs)
add_dependencies(string_player db_player)
add_dependencies(tf_player db_player)
add_dependencies(image_player db_player)
add_dependencies(jointstate_player db_player)
add_dependencies(gazebo_to_tf_player db_player)

target_link_libraries(
	db_player
  ${catkin_LIBRARIES}
  mongoclient
  ${Boost_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	mongodb_play
  db_player
  string_player
	tf_player
	image_player
	jointstate_player
  gazebo_to_tf_player
  ${catkin_LIBRARIES}
  mongoclient
  ${Boost_LIBRARIES}
)
