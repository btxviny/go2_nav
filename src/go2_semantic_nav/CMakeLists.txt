cmake_minimum_required(VERSION 3.8)
project(go2_semantic_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/build_semantic_map.py
  scripts/semantic_goal.py
  scripts/visualize_semantic_map.py
  DESTINATION lib/${PROJECT_NAME}
)

# Not a launchable entry point -- imported by build_semantic_map.py/semantic_goal.py/
# visualize_semantic_map.py as `import semantic_map_common`, which resolves because
# Python auto-prepends the running script's own directory to sys.path. Installed as
# FILES (not PROGRAMS) into the same lib/${PROJECT_NAME} dir as those scripts so that
# import works from the installed copies too, not just when run from source.
install(FILES
  scripts/semantic_map_common.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
