# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(monolithic_pr2_planner_node)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(catkin REQUIRED COMPONENTS
    monolithic_pr2_planner leatherman pcl_ros geometry_msgs sensor_msgs
    arm_navigation_msgs costmap_2d sbpl_manipulation_components_pr2
    interactive_markers sbpl_mha message_generation pviz tf2 pcl_conversions
    gridmap_2d
    )
find_package(orocos_kdl REQUIRED)
find_package(OpenCV REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES GetMobileArmPlan.srv
)

generate_messages(
    DEPENDENCIES trajectory_msgs geometry_msgs std_msgs 
)

catkin_package(
    DEPENDS orocos_kdl 
    CATKIN_DEPENDS roscpp geometry_msgs monolithic_pr2_planner costmap_2d
        pcl_conversions tf2 message_runtime pviz
        sbpl_manipulation_components_pr2 leatherman pcl_ros  geometry_msgs
        sensor_msgs arm_navigation_msgs gridmap_2d
        sbpl_manipulation_components_pr2 interactive_markers sbpl_mha
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

set(CATKIN_BUILD_TYPE Release )

SET(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")

# find_package(Boost 1.46.0 COMPONENTS thread system filesystem REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ompl REQUIRED ompl)
include_directories(${ompl_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})
link_directories(${ompl_LIBRARY_DIRS} )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${Boost_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS} ${ompl_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${OpenCV}_INCLUDE_DIRS)

add_executable(node src/node.cpp src/EnvInterfaces.cpp #(Not required for simulation?)
                             src/CollisionSpaceInterface.cpp
                             src/ompl_pr2_planner.cpp
                             src/ompl_collision_checker.cpp
                        )

target_link_libraries(node ${catkin_LIBRARIES} ${ompl_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(testPlanningRequest src/testPlanningRequest.cpp)
target_link_libraries(testPlanningRequest ${catkin_LIBRARIES})

add_executable(interactFullBodyPlanner src/interactFullBodyPlanner.cpp)
add_dependencies(interactFullBodyPlanner interactFullBodyPlanner_gen_cpp)
target_link_libraries(interactFullBodyPlanner
    ${catkin_LIBRARIES} ${ompl_LIBRARIES})

add_executable(testExecute src/testExecute.cpp)
add_dependencies(testExecute ${catkin_EXPORTED_TARGETS})
target_link_libraries(testExecute ${catkin_LIBRARIES})

add_executable(stlToOctomap src/publishSTLToOctomap.cpp)
target_link_libraries(stlToOctomap ${catkin_LIBRARIES})

add_executable(run_simulation src/run_simulations.cpp)
target_link_libraries(run_simulation ${catkin_LIBRARIES})

add_executable(run_sim_file src/runSimulationFromFile.cpp)
target_link_libraries(run_sim_file ${catkin_LIBRARIES})

add_executable(run_demo src/run_demo.cpp)
target_link_libraries(run_demo ${catkin_LIBRARIES})

add_executable(runTests src/run_tests.cpp)
target_link_libraries(runTests ${catkin_LIBRARIES})

add_executable(generateExperimentsNode src/generateExperimentsNode.cpp)
add_dependencies(generateExperimentsNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(generateExperimentsNode ${catkin_LIBRARIES})

add_executable(saveProjectedMap src/saveProjectedMap.cpp)
target_link_libraries(saveProjectedMap ${catkin_LIBRARIES} ${OpenCV_LIBS})
