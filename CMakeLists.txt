cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)

project(irsl_choreonoid)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if($ENV{ROS_VERSION} EQUAL 2)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(irsl_choreonoid)
else()

endif()

find_package(choreonoid REQUIRED)

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

if($ENV{ROS_VERSION} EQUAL 2)
include_directories(
  include
  ${CHOREONOID_INCLUDE_DIRS}
)
else()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)
endif()

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

#add_subdirectory(sample)
add_subdirectory(pybind11)

if($ENV{ROS_VERSION} EQUAL 2)
#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY sample
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY settings
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_include_directories(include)
ament_package()
else()
# for python module
catkin_python_setup()
#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY settings
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
endif()
