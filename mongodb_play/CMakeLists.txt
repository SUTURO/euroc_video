cmake_minimum_required(VERSION 2.8.3)
project(mongodb_play)

find_package(catkin REQUIRED roslib rostopic tf sensor_msgs)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)

#find_package(MongoClient REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES db_player
  CATKIN_DEPENDS roscpp
#  DEPENDS MongoClient
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
#  ${MongoClient_INCLUDE_DIR}
)

link_directories(${catkin_LINK_DIRS})
#link_directories(${MongoClient_LINK_DIRS})

add_library(db_player src/db_player.cpp )

## Declare a cpp executable
add_executable(mongodb_play src/mongodb_play.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(mongodb_play_node mongodb_play_generate_messages_cpp)

target_link_libraries(db_player
#  ${MongoClient_LIBRARIES}
  ${catkin_LIBRARIES}
  mongoclient
  ${Boost_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(mongodb_play
  db_player
  ${catkin_LIBRARIES}
  mongoclient
  ${Boost_LIBRARIES}
)
