cmake_minimum_required(VERSION 3.5.1) 
project(dbt_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp std_msgs
    behaviortree_cpp_v3
    actionlib_msgs
    actionlib
    message_generation
    geometry_msgs
    asar_hybrid_tmp
    )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
  FdInput.msg
)

add_service_files(
   FILES
   GetBT.srv
   SendTree.srv
   GetPddl.srv
   SuturePoints.srv
 )

add_action_files(
   DIRECTORY action
   FILES     
   BTExecutor.action 
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS} )

######################################################

add_executable(dbt_executor src/dbt_executor.cpp)
target_link_libraries(dbt_executor ${catkin_LIBRARIES} )
