cmake_minimum_required(VERSION 3.0.2)
project(ping360)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        Original_msgs
        sensor_msgs
        serial
        cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ping360
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(ping360_node
        src/ping360_node.cpp
        include/ping360_node.h
        src/pingImage_node.cpp
        include/pingImage_node.h )

target_link_libraries(ping360_node
        ${catkin_LIBRARIES}
        )

add_executable(toScan_node
        src/Ping360_Toscan_Node.cpp
        include/Ping360_Toscan_Node.h
        )

target_link_libraries(toScan_node
        ${catkin_LIBRARIES}
        )

add_executable(pingImage_node
        src/pingImage_node.cpp
        include/pingImage_node.h
        )

target_link_libraries(pingImage_node
        ${catkin_LIBRARIES}
        )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
