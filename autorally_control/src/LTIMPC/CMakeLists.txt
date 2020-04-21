#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)
add_executable(LTIMPC LTIMPC.cpp solver.c ldl.c matrix_support.c util.c)
add_dependencies(LTIMPC autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(LTIMPC ${include} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  LTIMPC
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
