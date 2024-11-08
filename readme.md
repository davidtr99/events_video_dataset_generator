# Events Video Dataset Generator
This package generates a dataset of events and photographs from a rosbag file. The main purpose of this package is to generate a dataset for training a neural network to predict the next frame of a video from a sequence of events.

This program takes a rosbag file, and reads chronologically the events and the images stored in it. It creates a folder with a specified name, and stored the events and the images in it. The events are stored in a .cvs file, and the images are stored in a .png file.

The generated files will have, for example, the following structure:
```
testbed_shelves_0000_000001123785.png
testbed_shelves_0000.csv
testbed_shelves_0001_000035510802.png
testbed_shelves_0001.csv
testbed_shelves_0002_000068894880.png
...
```

The file `testbed_shelves_0000.csv` contains the events that happened between the time `000001123785` and `000035510802`. The file `testbed_shelves_0000_000001123785.png` and the file `testbed_shelves_0001_000035510802.png` are the images before and after the events in that event file. And so on.

## Dependencies
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
- [dvs_msgs](https://github.com/davidtr99/dvs_msgs)

## Usage
To run the data generation node, after configuring the launchfile properly run the following command:
```
roslaunch events_video_dataset_generator node.launch
```

## Troubleshooting
Contact: David Tejero-Ruiz (dtejero@catec.aero)

Found a bug? Create an ISSUE!

Do you want to contribute? Create a PULL-REQUEST!