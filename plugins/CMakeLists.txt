
set(sources
  RobotAssemblerPlugin.cpp
  AssemblerBodyItem.cpp
  AssemblerFileIO.cpp
  AssemblerSceneBody.cpp
  AssemblerView.cpp
  AssemblerBar.cpp
  RobotAssembler.cpp
  RobotAssemblerHelper.cpp
  )

set(headers
  RobotAssemblerPlugin.h
  AssemblerBodyItem.h
  AssemblerFileIO.h
  AssemblerSceneBody.h
  AssemblerView.h
  AssemblerBar.h
  RobotAssembler.h
  RobotAssemblerHelper.h
  )

set(target CnoidRobotAssemblerPlugin)
#<#choreonoid_make_gettext_mo_files(${target} mofiles)
#<#qt5_add_resources(RC_SRCS BodyPlugin.qrc)
include_directories(${CHOREONOID_INCLUDE_DIRS})
choreonoid_add_plugin(${target} ${sources} HEADERS ${headers})

#<#set(boost_libraries ${Boost_IOSTREAMS_LIBRARY})

#<#target_link_libraries(${target} CnoidBody ${boost_libraries})
target_link_libraries(${target} CnoidBody CnoidUtil)

set_target_properties(${target}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

# add_subdirectory(pybind11)

### test
add_executable(test_yaml test_yaml.cpp RobotAssembler.cpp)
target_link_libraries(test_yaml CnoidUtil)
target_include_directories(test_yaml PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS})
