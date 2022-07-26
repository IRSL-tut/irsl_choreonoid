
set(sources
  IRSLXXXPlugin.cpp
  XXXBodyItem.cpp
  XXXFileIO.cpp
  XXXSceneBody.cpp
  )

set(headers
  IRSLXXXPlugin.h
  XXXBodyItem.h
  XXXFileIO.h
  XXXSceneBody.h
  )

set(target CnoidIRSLXXXPlugin)
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
