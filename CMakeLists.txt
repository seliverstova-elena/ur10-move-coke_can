cmake_minimum_required(VERSION 2.8.3)
project(ur10)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++0x") # Need C++11 

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   geometry_msgs
   message_generation
   moveit_commander
   moveit_msgs
   moveit_ros_planning_interface
   moveit_core
   gazebo_ros
)

find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_service_files(DIRECTORY srv FILES model_collision_bounding_box.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      std_msgs
      geometry_msgs
      message_runtime
    DEPENDS
      gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(UR10ModelCollisionPlugin SHARED plugins/model_collision_plugin.cc)
target_link_libraries(UR10ModelCollisionPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(UR10OperationsManager src/operations_manager.cpp)
target_link_libraries(UR10OperationsManager ${catkin_LIBRARIES})

add_library(UR10MotionControl src/motion_control.cpp )
target_link_libraries(UR10MotionControl ${catkin_LIBRARIES})

add_library(UR10Collision src/collision.cpp )
target_link_libraries(UR10Collision ${catkin_LIBRARIES})

add_library(UR10Scene src/scene.cpp )
target_link_libraries(UR10Scene ${catkin_LIBRARIES})

add_library(UR10Transform src/transform.cpp )
target_link_libraries(UR10Transform ${catkin_LIBRARIES})

add_executable(ur10_main_node src/main.cpp)
target_link_libraries(ur10_main_node ${catkin_LIBRARIES} 
                                     UR10OperationsManager
                                     UR10MotionControl 
                                     UR10Collision 
                                     UR10Scene
                                     UR10Transform)
                                                            
#add_dependencies(main_node ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(planner_node src/planner.cpp)
#target_link_libraries(planner_node ${catkin_LIBRARIES})
#add_dependencies(planner_node ${PROJECT_NAME}_generate_messages_cpp)
