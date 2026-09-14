cmake_minimum_required(VERSION 3.8)
project(go2_nav_bringup)

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
  scripts/send_nav_goal.py
  scripts/run_stack.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
