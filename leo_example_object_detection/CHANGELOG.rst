^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_example_object_detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2022-11-22)
------------------
* object_detection: object_detector: changed compression format of image_compressed to jpeg
* fixed grammar errors in package.xml descriptions for object_detection and line_follower packages
* Contributors: Aleksander Szymański

0.1.0 (2022-05-26)
------------------
* Object Detection example (`#2 <https://github.com/LeoRover/leo_examples/issues/2>`_)
  * initial commit with base functionalities of the node
  * object_detector.py: switched use of hardcoded input img shape, to getting the shape from loaded model params
  * object_detector.py: added prinitng confidence next to detection label, and fixed bugs with getting input shape
  * object_detector.py: remove debug prints
  * added specific colors for specific labels, both configurable in label_colors.yaml file
  * object_detector.py: drawing boxes on the unscalled image - added methods for scaling coordinates of the boxes
  * object_detector.py: fixed color channels, and made labels bigger
  * object_detector.py: used easier formula for translating bounding box coordinates
  * implemented guidlines from code review
  * package.xml: changed package author
  * detector.launch: added ros argument for chosing file with color definitions for labels
  * object_detector.py: moved redundant lines outisde try-catch statement
* Contributors: Aleksander Szymański
