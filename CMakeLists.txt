cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)

project(irsl_choreonoid)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

##find_package(catkin REQUIRED COMPONENTS xxxx)
find_package(catkin REQUIRED)
find_package(choreonoid REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES irsl_choreonoid
  #CATKIN_DEPENDS choreonoid
  DEPENDS CHOREONOID
)

### debug print variables
##get_cmake_property(_variableNames CACHE_VARIABLES)
#get_cmake_property(_variableNames VARIABLES)
#message("VARIABLES>>>>")
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#  message("${_variableName}=${${_variableName}}")
#endforeach()
#message("VARIABLES<<<<")

# set path for installing jsk_choreonoid libraries in choreonoid
set(choreonoid_PLUGINDIR ${CHOREONOID_PLUGIN_DIR})
set(choreonoid_PYTHON_DIR ${CHOREONOID_ROOT_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid)

message("********************choreonoid pkg infomation********************")
message("CHOREONOID_FOUND: ${CHOREONOID_FOUND}")
message("CHOREONOID_VERSION_STRING: ${CHOREONOID_VERSION_STRING}")
message("CHOREONOID_VERSION_MAJOR: ${CHOREONOID_VERSION_MAJOR}")
message("CHOREONOID_VERSION_MINOR: ${CHOREONOID_VERSION_MINOR}")
message("CHOREONOID_VERSION_PATCH: ${CHOREONOID_VERSION_PATCH}")
message("CHOREONOID_INCLUDE_DIRS: ${CHOREONOID_INCLUDE_DIRS}")
message("CHOREONOID_LIBRARY_DIRS: ${CHOREONOID_LIBRARY_DIRS}")
message("CHOREONOID_LIBRARIES: ${CHOREONOID_LIBRARIES}")
message("CHOREONOID_PLUGIN_DIR: ${CHOREONOID_PLUGIN_DIR}")
message("CHOREONOID_ADDITIONAL_LIBRARIES: ${CHOREONOID_ADDITIONAL_LIBRARIES}")
message("CHOREONOID_COMPILE_DEFINITIONS: ${CHOREONOID_COMPILE_DEFINITIONS}")

# apply choreonoid cmake settings
add_definitions(${CHOREONOID_DEFINITIONS})

#include_directories(${CHOREONOID_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

link_directories(${CHOREONOID_LIBRARY_DIRS})

### 相対パスを絶対パスに直す
##set(CHOREONOID_BODY_LIBRARIES_ABS)
##foreach(lib ${CHOREONOID_BODY_LIBRARIES})
##  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
##  if (NOT (${lib}_abs))
##    set(${lib}_abs ${lib})
##  endif()
##  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
##endforeach(lib)
##message ("cnoid_body_lib: ${CHOREONOID_BODY_LIBRARIES_ABS}")

# for python module
catkin_python_setup()


#add_library(${PROJECT_NAME}
#  src/EigenUtil.cpp
#)
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${CHOREONOID_BODY_LIBRARIES}
#  )

#############
## Install ##
#############
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

add_subdirectory(plugins)
add_subdirectory(pybind11)

