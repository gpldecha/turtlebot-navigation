cmake_minimum_required(VERSION 2.8.3)
project(topological_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
            roscpp
            tf
            nav_core
            costmap_2d
            base_local_planner
            pluginlib
            optitrack_rviz
            cmake_modules
            message_generation
            std_msgs
)

add_service_files(
   FILES
   String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()


find_package(Armadillo)
if(Armadillo_FOUND)
    message("Found Armadillo.")
    message("Armadillo include dir is: ${ARMADILLO_INCLUDE_DIR}")
    message("Armadillo lib's to be linked against: ${Armadillo_LIBRARIES}")
    message("Armadillo lib version: ${PACKAGE_FIND_VERSION}")
endif(Armadillo_FOUND)

add_definitions(-std=c++11)
add_definitions(${EIGEN_DEFINITIONS})


set(INCLUDE_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/include     )
set(SRC_DIR             ${CMAKE_CURRENT_SOURCE_DIR}/src         )


set(INC_HEADER          ${INCLUDE_DIR}/topological_map/costmap2topology.h
                        ${INCLUDE_DIR}/topological_map/rviz_topology.h
                        ${INCLUDE_DIR}/topological_map/topologymap.h
                        ${INCLUDE_DIR}/topological_map/occupancygrid_listener.h
                        ${INCLUDE_DIR}/online_search/online_search.h
                        ${INCLUDE_DIR}/online_search/utils.h
                        ${INCLUDE_DIR}/online_search/objective_functions.h
                        ${INCLUDE_DIR}/finite_state_machine/finite_state_machine.h
                        ${INCLUDE_DIR}/agent/agent.h


)

set(SRC_FILE            ${SRC_DIR}/topological_map/costmap2topology.cpp
                        ${SRC_DIR}/topological_map/rviz_topology.cpp
                        ${SRC_DIR}/topological_map/occupancygrid_listener.cpp
                        ${SRC_DIR}/online_search/online_search.cpp
                        ${SRC_DIR}/finite_state_machine/finite_state_machine.cpp
                        ${SRC_DIR}/agent/agent.cpp
)


include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS}  ${ARMADILLO_INCLUDE_DIR})

## Declare a C++ library
add_library(topological_map ${SRC_FILE} ${INC_HEADER})
target_link_libraries(topological_map ${catkin_LIBRARIES} ${Armadillo_LIBRARIES})



## Declare a C++ executable
add_executable(one_agent_node src/nodes/one_agent_node.cpp)
target_link_libraries(one_agent_node topological_map ${catkin_LIBRARIES} ${Armadillo_LIBRARIES})

add_executable(two_agents_node src/nodes/two_agents_node.cpp)
target_link_libraries(two_agents_node topological_map ${catkin_LIBRARIES} ${Armadillo_LIBRARIES})


add_executable(test_online_search src/test/test_online_search.cpp)
target_link_libraries(test_online_search topological_map ${catkin_LIBRARIES} ${Armadillo_LIBRARIES})









