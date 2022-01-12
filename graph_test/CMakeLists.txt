cmake_minimum_required(VERSION 3.0.2)
project(graph_test)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)


set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp graph_core
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graph_test
  CATKIN_DEPENDS roscpp graph_core
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#add_executable(       test_collision src/test_collision.cpp)
#add_dependencies(     test_collision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_collision
#  ${catkin_LIBRARIES}
#)


add_executable(       test_subtree src/test_subtree.cpp)
add_dependencies(     test_subtree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_subtree
  ${catkin_LIBRARIES}
)

#add_executable(       test_precomputing_tree src/test_precomputing_tree.cpp)
#add_dependencies(     test_precomputing_tree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_precomputing_tree
#  ${catkin_LIBRARIES}
#)

#add_executable(       narrow_pass_test src/narrow_pass_test1.cpp)
#add_dependencies(     narrow_pass_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(narrow_pass_test
#  ${catkin_LIBRARIES}
#)


#add_executable(       highdof_test src/highdof_test.cpp)
#add_dependencies(     highdof_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(highdof_test
#  ${catkin_LIBRARIES}
#)

add_executable(       test_multigoal src/test_multigoal.cpp)
add_dependencies(     test_multigoal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_multigoal
  ${catkin_LIBRARIES}
)

add_executable(       test_datastructure src/test_datastructure.cpp)
add_dependencies(     test_datastructure ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_datastructure
  ${catkin_LIBRARIES}
)
#install(PROGRAMS
#  scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
