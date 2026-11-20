cmake_minimum_required(VERSION 3.16)
project(robot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY ./ DESTINATION share/${PROJECT_NAME}
  PATTERN CMakeLists.txt EXCLUDE PATTERN package.xml EXCLUDE)
ament_package()
