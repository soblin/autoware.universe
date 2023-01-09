cmake_minimum_required(VERSION 3.14)
project(tier4_localization_rviz_plugin)

find_package(autoware_cmake REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)
find_package(rviz_ogre_vendor REQUIRED)
autoware_package()

set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_history/pose_history_display.hpp
  src/pose_history/pose_history_display.cpp
  src/pose_history_footprint/display.hpp
  src/pose_history_footprint/display.cpp
)

# NOTE(Mamoru Sobue): workaround to suprress compilation warning for rviz_ogre_vendor
get_target_property(Ogre_INCLUDE_DIRS rviz_ogre_vendor::OgreMain INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Ogre_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
