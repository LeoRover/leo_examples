cmake_minimum_required(VERSION 3.0.2)
project(leo_example_object_detection)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Confidence.cfg
)


catkin_package(CATKIN_DEPENDS
  dynamic_reconfigure
)

install(
  DIRECTORY 
    config
    launch
    models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(
  PROGRAMS scripts/object_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)