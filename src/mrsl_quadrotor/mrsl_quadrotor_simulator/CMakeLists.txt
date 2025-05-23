cmake_minimum_required(VERSION 2.8.12)
project(mrsl_quadrotor_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros gazebo_dev quadrotor_msgs mavros_msgs)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

catkin_package(CATKIN_DEPENDS roscpp gazebo_ros quadrotor_msgs mavros_msgs)

add_library(mrsl_quadrotor_propulsion SHARED src/quadrotor_propulsion.cpp
  src/Quadrotor.cpp
  src/QuadrotorSO3AttitudeControl.cpp)
target_link_libraries(mrsl_quadrotor_propulsion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mrsl_quadrotor_propulsion ${catkin_EXPORTED_TARGETS})

add_executable(transform_to_my_controller src/transform_to_my_controller.cpp)
target_link_libraries(transform_to_my_controller ${catkin_LIBRARIES})

add_executable(transform_to_my_controller2 src/transform_to_my_controller2.cpp)
target_link_libraries(transform_to_my_controller2 ${catkin_LIBRARIES})
 