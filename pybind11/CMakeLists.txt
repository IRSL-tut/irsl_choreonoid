set(TARGET PyIRSLUtil)

add_cnoid_python_module(${TARGET} PyIRSLUtil.cpp)

target_link_libraries(${TARGET}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  ${PROJECT_NAME}
  )

target_compile_definitions(${TARGET} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
