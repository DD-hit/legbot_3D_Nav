cmake_minimum_required(VERSION 3.16)
project(legbot_bringup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(legbot_platform)
find_package(legbot_runtime REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
foreach(n reference_path_transform fastlio_odom_adapter cloud_transform)
add_executable(${n} src/${n}.cpp)
target_compile_features(${n} PUBLIC cxx_std_17)
ament_target_dependencies(${n} legbot_runtime tf2_sensor_msgs)
install(TARGETS ${n} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(PROGRAMS
  scripts/go2_command_adapter
  scripts/navigation_velocity_guard
  scripts/go2_estop_keyboard
  scripts/go2_demo_sequencer
  scripts/map_localizer
  scripts/pct_path_mission
  scripts/fastlio_truth_monitor
  scripts/scan_waypoint_mission
  scripts/stair_cloud_sim
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/map_match_core.py scripts/waypoint_tasks.py scripts/navigation_web.py scripts/floor_map.py DESTINATION lib/${PROJECT_NAME})
set(floor_display "${CMAKE_CURRENT_SOURCE_DIR}/web_maps/building2_9.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${floor_display}")
file(READ "${floor_display}" FLOOR_DISPLAY)
configure_file(web/index.html "${CMAKE_CURRENT_BINARY_DIR}/web/index.html" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/web/index.html" web/console.css web/console.js DESTINATION share/${PROJECT_NAME}/web)
find_package(legbot_config REQUIRED)
get_filename_component(settings "${legbot_config_DIR}/.." ABSOLUTE)
file(GLOB old_profiles CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/config/*.yaml")
foreach(profile IN LISTS old_profiles)
  get_filename_component(filename "${profile}" NAME)
  if(filename STREQUAL "map.yaml")
    legbot_resource_alias("config/map.yaml" "${settings}/maps/building2_9/map.yaml")
    continue()
  endif()
  get_filename_component(source "${profile}" REALPATH)
  file(RELATIVE_PATH resource "${CMAKE_CURRENT_SOURCE_DIR}/../../config" "${source}")
  legbot_resource_alias("config/${filename}" "${settings}/${resource}")
endforeach()
legbot_resource_alias("web_maps/building2_9.json" "${settings}/maps/building2_9/display.json")
legbot_resource_alias("worlds/Building.sdf" "${settings}/maps/building2_9/world.sdf")
install(DIRECTORY launch models worlds rviz web_maps DESTINATION share/${PROJECT_NAME}
  PATTERN "building2_9.json" EXCLUDE PATTERN "Building.sdf" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
