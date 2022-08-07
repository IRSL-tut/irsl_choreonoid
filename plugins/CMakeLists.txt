# boost
#set(Boost_USE_STATIC_LIBS OFF)
#set(Boost_ADDITIONAL_VERSIONS "1.62" "1.63" "1.64" "1.65" "1.66" "1.67" "1.68" "1.69" "1.70" "1.71" "1.72" "1.73" "1.74" "1.75")
#set(boost_packages program_options iostreams system)
#if(CMAKE_CXX_STANDARD LESS 17)
#  set(boost_packages ${boost_packages} filesystem)
#endif()
#if(MSVC)
#  find_package(Boost 1.58.0 REQUIRED COMPONENTS ${boost_packages} OPTIONAL_COMPONENTS bzip2 zlib)
#else()
#  find_package(Boost 1.58.0 REQUIRED COMPONENTS ${boost_packages})
#endif()
#
#if(CMAKE_CXX_STANDARD LESS 17)
#  set(FILESYSTEM_LIBRARY ${Boost_FILESYSTEM_LIBRARY})
#elseif(UNIX)
#  set(FILESYSTEM_LIBRARY stdc++fs)
#endif()
#
#set(boost_libraries ${Boost_FILESYSTEM_LIBRARY})
message("Boost: ${Boost_INCLUDE_DIRS}")
message("Boost: ${Boost_LIBRARY_DIRS}")
message("Boost: ${boost_libraries}")
#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})

set(sources
  RobotAssemblerPlugin.cpp
  AssemblerItem.cpp
  AssemblerView.cpp
  AssemblerTreeView.cpp
  AssemblerBar.cpp
  AssemblerManager.cpp
  RobotAssemblerSettings.cpp
  RobotAssembler.cpp
  RobotAssemblerBody.cpp
  RobotAssemblerHelper.cpp
  )

set(headers
  RobotAssemblerPlugin.h
  AssemblerItem.h
  AssemblerView.h
  AssemblerTreeView.h
  AssemblerBar.h
  AssemblerManager.h
  RobotAssemblerSettings.h
  RobotAssembler.h
  RobotAssemblerBody.h
  RobotAssemblerHelper.h
  )

set(target CnoidRobotAssemblerPlugin)
#<#choreonoid_make_gettext_mo_files(${target} mofiles)
#<#qt5_add_resources(RC_SRCS BodyPlugin.qrc)
include_directories(${CHOREONOID_INCLUDE_DIRS})
choreonoid_add_plugin(${target} ${sources} HEADERS ${headers})

#[TODO] boost_filesystem??
target_link_libraries(${target} CnoidBody CnoidUtil boost_filesystem) ## RobotAssembler

set_target_properties(${target}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

# add_subdirectory(pybind11)
#add_library(libRobotAssembler RobotAssemblerSettings.cpp RobotAssembler.cpp)
### test
add_executable(test_yaml test_yaml.cpp RobotAssemblerSettings.cpp RobotAssembler.cpp RobotAssemblerBody.cpp)
target_link_libraries(test_yaml CnoidUtil CnoidBody)
target_include_directories(test_yaml PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS})

add_executable(test_robot test_robot.cpp)
target_link_libraries(test_robot CnoidUtil CnoidBody)
target_include_directories(test_robot PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS})
