# these plugins should be uploaded to upstream repository
# https://github.com/ros-visualization/rviz/pull/634

# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(local_overlay_text)
if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
  add_compile_options(-std=c++11)
endif()
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rviz 
  message_generation std_msgs 
  geometry_msgs)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

add_message_files(FILES
  Text.msg
  TextArray.msg)
#dd_service_files(FILES
#  RequestMarkerOperate.srv EusCommand.srv
#  Screenshot.srv
#  )
#catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    DEPENDS rviz
    CATKIN_DEPENDS 
    INCLUDE_DIRS # TODO include
    LIBRARIES ${PROJECT_NAME}
)


set(ROS_BUILD_TYPE Release)

include_directories(src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set(SOURCE_FILES src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)

set(HEADER_FILES
  # src/local_overlay_text_display.h
  src/local_overlay_text_array_display.h
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

set(SOURCE_FILES
  # src/local_overlay_text_display.cpp
  src/local_overlay_text_array_display.cpp
  src/overlay_utils.cpp
  ${MOC_FILES}
)

add_library(local_overlay_text ${SOURCE_FILES} ${UIC_FILES})
if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(local_overlay_text ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(local_overlay_text Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()
add_dependencies(local_overlay_text
  ${PROJECT_NAME}_generate_messages_cpp)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(local_overlay_text PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

add_dependencies(local_overlay_text ${PROJECT_NAME}_generate_messages_cpp )
add_dependencies(local_overlay_text ${PROJECT_NAME}_generate_messages )

# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(TARGETS local_overlay_text
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY cfg config icons launch samples test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
