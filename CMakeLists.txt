cmake_minimum_required(VERSION 3.15.0)
project(mrs_errorgraph)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-save-temps)
# add_definitions(-O0)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  message_generation
  nodelet
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# messages
add_message_files(DIRECTORY msg FILES
  ErrorgraphNodeID.msg
  ErrorgraphError.msg
  ErrorgraphElement.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  )

set(LIBRARIES
  MrsErrorgraph
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## | ------------ MRSErrorgraphViewer ------------ |

## Declare a C++ library
add_library(MrsErrorgraph
  src/errorgraph.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsErrorgraph
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/mrs_errorgraph/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
