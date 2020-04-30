#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)

add_library(LTIMPC_functions LTIMPC.cpp solver.c ldl.c matrix_support.c util.c VehDyn.cpp)
add_dependencies(LTIMPC_functions autorally_msgs_gencpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(LTIMPC_functions ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(LTIMPC_node LTIMPC_main.cpp)
add_dependencies(LTIMPC_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(LTIMPC_node LTIMPC_functions)



install(TARGETS
  LTIMPC_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
