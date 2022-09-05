cmake_minimum_required(VERSION 3.0.2)
project(sentech_gige_driver)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  camera_info_manager
)

find_package(StApi REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sentech_gige_driver
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

message(STATUS "StApi_INCLUDE_DIRS:${StApi_INCLUDE_DIRS}")
message(STATUS "StApi_LIBRARIES   :${StApi_LIBRARIES}")

include_directories(
#  include
  ${StApi_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_executable(sentech_gige_driver
  src/sentech_gige_driver.cpp
)
target_link_libraries(sentech_gige_driver
  ${StApi_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
