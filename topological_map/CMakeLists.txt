cmake_minimum_required(VERSION 2.8.3)
project(topological_map)

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

)

set(SRC_FILE            ${SRC_DIR}/topological_map/costmap2topology.cpp
                        ${SRC_DIR}/topological_map/rviz_topology.cpp
                        ${SRC_DIR}/topological_map/occupancygrid_listener.cpp
                        ${SRC_DIR}/online_search/online_search.cpp
                        ${SRC_DIR}/finite_state_machine/finite_state_machine.cpp

)


include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS}  ${ARMADILLO_INCLUDE_DIR})

## Declare a C++ library
add_library(topological_map ${SRC_FILE} ${INC_HEADER})
target_link_libraries(topological_map ${catkin_LIBRARIES} ${Armadillo_LIBRARIES})



## Declare a C++ executable
add_executable(topological_map_node src/nodes/topological_map_node.cpp)
target_link_libraries(topological_map_node topological_map ${catkin_LIBRARIES} ${Armadillo_LIBRARIES})


add_executable(test_online_search src/test/test_online_search.cpp)
target_link_libraries(test_online_search topological_map ${catkin_LIBRARIES} ${Armadillo_LIBRARIES})

