cmake_minimum_required(VERSION 3.8)
project(go2_office_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  models
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/frontier_explorer.py
  scripts/record_bag.sh
  scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
