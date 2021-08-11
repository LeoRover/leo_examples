cmake_minimum_required(VERSION 3.0.2)
project(leo_examples)

find_package(catkin)

catkin_package()

install(
  DIRECTORY 
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/follow_ar_tag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)