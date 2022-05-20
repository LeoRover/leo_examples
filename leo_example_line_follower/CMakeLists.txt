cmake_minimum_required(VERSION 3.0.2)
project(leo_example_line_follower)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/ColorMask.cfg
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
  PROGRAMS scripts/data_saver.py
           scripts/line_follower.py
           scripts/color_mask.py
           scripts/prepare_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)