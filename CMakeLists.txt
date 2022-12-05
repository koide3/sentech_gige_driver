cmake_minimum_required(VERSION 3.5.2)
project(sentech_gige_driver)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    image_transport
    camera_info_manager
  )
  catkin_package()
else()
  # ROS2
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
endif()


find_package(StApi REQUIRED)
find_package(OpenCV REQUIRED)


###########
## Build ##
###########

message(STATUS "StApi_INCLUDE_DIRS:${StApi_INCLUDE_DIRS}")
message(STATUS "StApi_LIBRARIES   :${StApi_LIBRARIES}")

if($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
  add_executable(sentech_gige_driver
    src/sentech_gige_driver_ros1.cpp
  )
  target_include_directories(sentech_gige_driver PUBLIC
    include
    ${StApi_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(sentech_gige_driver
    ${StApi_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
  )
else()
  # ROS2
  ament_auto_add_executable(sentech_gige_driver
    src/sentech_gige_driver_ros2.cpp
  )
  target_include_directories(sentech_gige_driver PUBLIC
    include
    ${StApi_INCLUDE_DIRS}
  )
  target_link_libraries(sentech_gige_driver
    ${StApi_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

  ament_auto_package()
endif()
