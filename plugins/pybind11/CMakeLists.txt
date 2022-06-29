###
set(TARGET PyIRSLSimPlugin)

add_cnoid_python_module(${TARGET} PyIRSLSimPlugin.cpp)

target_link_libraries(${TARGET}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  CnoidIRSLSimPlugin
  )

target_compile_definitions(${TARGET} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})

message("CHOREONOID_BIN_DIR: ${CHOREONOID_BIN_DIR}")
if ("${CHOREONOID_BIN_DIR}" MATCHES ".*install.*")
  ## install
  message("in INSTALLSPACE")
  install(TARGETS ${TARGET} LIBRARY DESTINATION ${choreonoid_PYTHON_DIR})
else()
  ## devel
  message("in DEVELSPACE")
  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
endif()
