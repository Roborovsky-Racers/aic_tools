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

ament_auto_add_executable(gnss_filter
  src/gnss_filter.cpp
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/obstacle_parser.py
  scripts/publish_initialpose.py
  scripts/trim_bag.py
  scripts/analyze_bag.py
  scripts/analyze_bag.bash
  scripts/auto_mode_period_marker.py
  scripts/velocity_heat_odom_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# create python virtual environment
execute_process(
  COMMAND /usr/bin/python3 -m venv ${CMAKE_INSTALL_PREFIX}/.venv/analyze_bag
  RESULT_VARIABLE venv_result
)
if(venv_result)
    message(FATAL_ERROR "Failed to create virtual environment")
endif()

# install python dependencies in virtual environment
execute_process(
    COMMAND ${CMAKE_INSTALL_PREFIX}/.venv/analyze_bag/bin/pip install -r ./resources/requirements/analyze_bag.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE install_result
)
if(install_result)
    message(FATAL_ERROR "Failed to install python dependencies in virtual environment")
endif()

ament_auto_package(INSTALL_TO_SHARE
  resources
)