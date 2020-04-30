#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)

add_library(LTVMPC_functions LTVMPC.cpp solver.c ldl.c matrix_support.c util.c VehDyn.cpp)
add_dependencies(LTVMPC_functions autorally_msgs_gencpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(LTVMPC_functions ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(LTVMPC_node LTVMPC_main.cpp)
add_dependencies(LTVMPC_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(LTVMPC_node LTVMPC_functions)



install(TARGETS
  LTVMPC_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
