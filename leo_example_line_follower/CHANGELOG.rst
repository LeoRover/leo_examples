^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_example_line_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2022-11-22)
------------------
* fixed grammar errors in package.xml descriptions for object_detection and line_follower packages
* Contributors: Aleksander Szymański

0.1.0 (2022-05-26)
------------------
* Line follower example (`#1 <https://github.com/LeoRover/leo_examples/issues/1>`_)
  * initial commit for line follower package
  * package.xml: remove redundant dependencies
  * data_saver.py: replaced use od ros parameters with command line arguments and python argparse
  * data_saver.py: added info on node's exit and fixed path for file with labels
  * data_saver.py: Added info logs for creating files and directories
  * data_saver.py: starting time counting with the first message from cmd_vel topic, fixed saved img filename, closing label file on shutdown
  * data_saver.py: added saving images and labels
  * data_saver.py: added ignoring images with label (0.0, 0.0)
  * added line_follower.py - script for autonomous following the line with rover
  * added directory with first models, needed for line_follower.py script
  * models: added model trained on color masks, with two lines track
  * color_mask: added color_mask.py script with ros node for chosing hsv bounds for color mask
  * color_mask: added cfg directory with yaml files with example hsv_bounds and dynamic_reconfigure config file
  * color_mask: changed package.xml and CMakeLists.txt for use of dynamic_reconfigure
  * data_saver.py: fixed typos and formatted code with black
  * data_saver: removed unused arguments from launch file
  * color mask: added launch file for the color_mask_finder node
  * line_follower.py: added use of ros parameters for hsv color bounds
  * line follower: added launch file for the line follower node
  * moved yaml files to config directory
  * fixed typos, and added descriptions to ros args
  * removed redundant files
  * line_follower.py: switched param names to private namespace
  * added model trained on more accurate color masks
  * CMakeLists.txt: fixed catkin_lint errors
  * package.xml: updated package description
  * color_mask.py: implemented code review guidlines (about python buildins and instance method)
  * data_saver.py: implemented guidlines from code review
  * scripts: implemented guidlines from code review
  * config: added yaml files with color mask values for blue and red colors
  * color_mask.launch: changed default value of 'file' arg
  * package.xml: fixed typo
  * data_saver.py: implemented guidelines from code review
  * scripts: added script for processing saved data into ready dataset for keras model
  * prepare_data.py: fixed os.join bugs
  * data_saver.py: added base path if output directory is given as relative path
  * data_saver.py: added use of ptahlib home function instead of hardcoding home directory path
  * prepare_data.py: made the script executable with rosrun
  * prepare_data.py: gave the script execution rights with chmod + x
  * prepare_data.py: removing parent directories when train and validation dirs are nested paths
  * prepare_data.py: switched moving and removing files to just copying
  * prepare_data.py: added creating of zip file from processed data, and clearing the working directory from zipped files
  * prepare_data.py: added printing info about zipping files
  * line_follower.launch: fixed ros find package syntax error
  * prepare_data.py: changed argument names to one pattern
  * prepare_data.py: added checking if the zipfile name ends with '.zip'
  * prepare_data.py: formated the code using black
  * added notebook to repository
  * CMakeLists.txt: added prepare_data.py script to catkin_install_python section
  * implemented code review guidlines
  * line_follower.launch: updated arg name in rosparam
* Contributors: Aleksander Szymański
