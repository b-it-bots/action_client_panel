cmake_minimum_required(VERSION 2.8.3)
project(action_client_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  actionlib_msgs
  mir_yb_action_msgs
)



include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
   src/action_client_panel.cpp
   src/clients/clientBase/ClientBase.cpp
   src/clients/MoveBaseClient.cpp
   src/clients/clientBase/PerceiveClientBase.cpp
   src/clients/PerceiveCavityClient.cpp
   src/clients/PerceiveLocationClient.cpp
   src/clients/clientBase/ObjectManipulationClientBase.cpp
   src/clients/PickObjectClient.cpp
   src/clients/PlaceObjectClient.cpp
   src/clients/clientBase/StagingClientBase.cpp
   src/clients/StageObjectClient.cpp
   src/clients/UnStageObjectClient.cpp
   src/clients/clientBase/InsertObjectClientBase.cpp
   src/clients/InsertObjectClient.cpp
   src/clients/InsertObjectInCavityClient.cpp
)

set(HEADER_FILES
   src/action_client_panel.h
   src/clients/clientBase/ClientBase.h
   src/clients/MoveBaseClient.h
   src/clients/clientBase/PerceiveClientBase.h
   src/clients/PerceiveCavityClient.h
   src/clients/PerceiveLocationClient.h
   src/clients/clientBase/ObjectManipulationClientBase.h
   src/clients/PickObjectClient.h
   src/clients/PlaceObjectClient.h
   src/clients/clientBase/StagingClientBase.h
   src/clients/StageObjectClient.h
   src/clients/UnStageObjectClient.h
   src/clients/clientBase/InsertObjectClientBase.h
   src/clients/InsertObjectClient.h
   src/clients/InsertObjectInCavityClient.h
)

 generate_messages(
 DEPENDENCIES actionlib_msgs std_msgs geometry_msgs  mir_yb_action_msgs 
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs mir_yb_action_msgs
)

add_executable(test_action_server src/test_action_server.cpp)

target_link_libraries(
  test_action_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  test_action_server
  ${action_client_plugin_EXPORTED_TARGETS}
)

add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${SDL2_LIBRARY} ${GLEW_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
