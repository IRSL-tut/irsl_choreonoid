###
set(TARGET PyIRSLUtil)

add_cnoid_python_module(${TARGET} PyIRSLUtil.cpp)

target_link_libraries(${TARGET}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
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

###
set(TARGET2 PyDrawInterface)

#set(headers DrawInterface.h)
add_cnoid_python_module(${TARGET2} PyDrawInterface.cpp)

target_link_libraries(${TARGET2}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BASE_LIBRARIES}
  )

target_compile_definitions(${TARGET2} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})

message("CHOREONOID_BIN_DIR: ${CHOREONOID_BIN_DIR}")
if ("${CHOREONOID_BIN_DIR}" MATCHES ".*install.*")
  ## install
  message("in INSTALLSPACE")
  install(TARGETS ${TARGET2} LIBRARY DESTINATION ${choreonoid_PYTHON_DIR})
else()
  ## devel
  message("in DEVELSPACE")
  set_target_properties(${TARGET2}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
endif()

###
set(TARGET3 PyAssimp)

#set(headers DrawInterface.h)
add_cnoid_python_module(${TARGET3} PyAssimpModule.cpp)

target_link_libraries(${TARGET3}
  ${CHOREONOID_BASE_LIBRARIES}
  CnoidAssimpSceneLoader
  )

target_compile_definitions(${TARGET3} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})

message("CHOREONOID_BIN_DIR: ${CHOREONOID_BIN_DIR}")
if ("${CHOREONOID_BIN_DIR}" MATCHES ".*install.*")
  ## install
  message("in INSTALLSPACE")
  install(TARGETS ${TARGET3} LIBRARY DESTINATION ${choreonoid_PYTHON_DIR})
else()
  ## devel
  message("in DEVELSPACE")
  set_target_properties(${TARGET3}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
endif()
