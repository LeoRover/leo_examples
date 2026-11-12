

```wiki
'''Note:''' You can also check a detailed integration tutorial on [[https://www.leorover.tech/integrations/line-follower|LeoRover Tech - Line follower|target="_blank"]]
```



## Overview

This package contains ROS nodes which enable a
[Leo Rover](Robots/Leo Rover) to perform a line follower task with the
use of a neural network model, that had been trained on a color mask
(capturing a line color from the camera). 

```wiki
'''Note:''' The model was trained to perform the task on a two-lined track, so it drives between the lines.
```

 In
the package you can find default configuration, which consists of: 1. A
few trained neural network models 2. A couple of predefined HSV bounds
in the *yaml* files for catching the color mask

The package also contains nodes, scripts, and notebooks prepared for
gathering and processing your training data, and training your own
model, if you would like to (detailed instructions about training, your
own model in the **note** above the contents).

It is one of a couple of packages made for showing an example usage of a
stock [Leo Rover](Robots/Leo Rover).

## Usage

### Line follower

To run the line follower node, simply type: 

```
roslaunch leo_example_line_follower line_follower.launch
```

 It
will run the default configuration for the line_follower node. However,
you can specify your configuration by using launch arguments, which are
as follows: color_mask_file:: The path to the yaml file with the HSV
bounds (ROS parameters) for catching the line color. pub_mask:: The flag
specifying whether or not to publish the color mask (neural network
model input). camera:: The name of the topic with the camera image - you
can specify which camera view you want to use for following a line.
vel:: The name of the topic to which the node will publish the Twist
messages. model:: The path to the neural network model.

### Color mask

To run the color mask finder node, simply type: 

```
roslaunch leo_example_line_follower color_mask.launch
```


Then you need to run rqt, and from the plugins choose: 1. The **Image
View** - to see the color mask for the current HSV bounds. 2. The
**Dynamic Reconfigure** - to choose the HSV bounds with the sliders.

After specifying your bounds, you can kill the node and it will print
your configuration on the standard output.

### Data saver

To run the data saver node, simply type: 

```
roslaunch leo_example_line_follower record_data.launch
```

 It
will run the default configuration for the node. Although, you can
specify your configuration, by using launch arguments, which are as
follows: duration:: The information on how long the data will be
recorded. output_dir:: The name of the output directory for the recorded
data. video_topic:: The name of the topic with the camera image - you
can specify which camera you want to record the data from. vel_topic::
The name of the topic with the rover velocity - the label for the
recorded data

### Preparing data

After recording the training data, you need to process it to be ready to
use in the training. Therefore, the package contains a script that takes
the names of the directories with the data (with partition for the
training and validation data), and returns a single zip file. To run the
script, simply type: 

```
rosrun leo_example_line_follower prepare_data.py -t <train_directories> -v <valid_directories> -z <zip_file>
```

 where: train_directories::
Are the names of the directories with the training data.
valid_directories:: Are the names of the directories with the validation
data. zip_file:: Is the name of the final zip file. Therefore, an
example usage of the script is: 

```
rosrun leo_example_line_follower prepare_data.py -t train1 train2 -v val1 val2 -z dataset.zip
```



## ROS API

### line_follower

#### Subscribed Topics

- `camera/image_raw`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The image from the camera that will be provided to the neural network
  model.

#### Published Topics

- `cmd_vel`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  The target velocity for the rover from the neural network model output

#### Parameters

- `pub_mask` (`bool`): the flag specifying whether or not the node will
  publish the color_mask (the model's input).
- `hue_max` (`int`, default: `179`): The maximum hue value for the
  color_mask.
- `hue_min` (`int`, default: `0`): The minimum hue value for the
  color_mask.
- `sat_max` (`int`, default: `255`): The maximum saturation value for
  the color_mask.
- `sat_min` (`int`, default: `0`): The minimum saturation value for the
  color_mask.
- `val_max` (`int`, default: `255`): The maximum val (V from HSV) value
  for the color_mask.
- `val_min` (`int`, default: `0`): The minimum val (V from HSV) value
  for the color_mask.

### color_mask

#### Subscribed Topics

- `camera/image_raw`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The image from the camera that will be used for catching a color with
  the color mask.

#### Published Topics

- `color_mask`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  A live view of the color mask caught with the currently chosen values.
- `catched_colors/compressed`
  ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)):
  The live view of the colors that were caught with the color mask.

#### Parameters

- `hue_max` (`int`, default: `179`): The maximum hue value for the
  color_mask.
- `hue_min` (`int`, default: `0`): The minimum hue value for the
  color_mask.
- `sat_max` (`int`, default: `255`): The maximum saturation value for
  the color_mask.
- `sat_min` (`int`, default: `0`): The minimum saturation value for the
  color_mask.
- `val_max` (`int`, default: `255`): The maximum val (V from HSV) value
  for the color_mask.
- `val_min` (`int`, default: `0`): The minimum val (V from HSV) value
  for the color_mask.

### data_saver

#### Subscribed Topics

- `camera/image_raw`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  The camera view that will be recorded.
- `cmd_vel`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  The current velocity of the rover, used as labels for a recorded image
