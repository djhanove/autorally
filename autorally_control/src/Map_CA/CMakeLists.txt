#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)

add_executable(Map_CA Map_CA.cpp)
add_dependencies(Map_CA autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(Map_CA ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  Map_CA
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
