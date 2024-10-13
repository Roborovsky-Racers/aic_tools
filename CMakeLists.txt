cmake_minimum_required(VERSION 3.5.0)
project(aic_tools)

# Default to C++23
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  aic_tools/bag_tools.py
  scripts/obstacle_parser.py
  scripts/publish_initialpose.py
  scripts/trim_bag.py
  scripts/control_mode_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()