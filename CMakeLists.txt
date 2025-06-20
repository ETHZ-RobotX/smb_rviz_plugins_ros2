cmake_minimum_required(VERSION 3.8)
project(smb_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add these lines before building your library
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/smb_rviz_plugins/smb_power_motor/smb_batt_mot_panel.hpp
  include/smb_rviz_plugins/smb_power_payload/smb_batt_payload_panel.hpp
)

# Add Qt resource compilation
qt5_add_resources(QRC_FILES
  resources/battery_icons.qrc
)

add_library(smb_battery_panels SHARED
  src/smb_power_motor/smb_batt_mot_panel.cpp
  src/smb_power_payload/smb_batt_payload_panel.cpp
  ${MOC_FILES}
  ${QRC_FILES}
)

target_include_directories(smb_battery_panels PUBLIC
  ${sensor_msgs_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(smb_battery_panels
  rviz_common
  pluginlib
  rclcpp
  sensor_msgs
)

target_link_libraries(smb_battery_panels
  Qt5::Widgets
)

# wtf I don't understand why we need this lol, but it wont build without it
install(
  TARGETS smb_battery_panels
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

# Add these lines after the install commands
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

add_executable(fake_battery_publisher
  src/fake_battery_publisher.cpp
)

target_link_libraries(fake_battery_publisher
  rclcpp::rclcpp
  rviz_common::rviz_common
)

target_include_directories(fake_battery_publisher PRIVATE
  ${sensor_msgs_INCLUDE_DIRS}
)

install(TARGETS fake_battery_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Add these after the other install commands
install(
  DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

ament_package()

# add_library(smb_rviz_plugins_ros2 SHARED
#   src/smb_power_motor/smb_batt_mot_panel.cpp
#   # src/smb_power_payload/smb_batt_payload_panel.cpp
#   ${MOC_FILES}
#   ${QRC_FILES}
# )
