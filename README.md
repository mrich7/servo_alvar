# servo_alvar
Contains the servo_alvar and image_sharper ROS packages

servo_alvar:
Code to servo an Alvar marker using the PR2 navigation stack, move_base package, and ar_track_alvar

Usage: 

    rosrun servo_alvar servo_alvar.py <number of Alvar marker found with ar_track_alvar>

Can be edited to servo to any AR tag that has a TF frame (assumes z-axis is pointing out of the face of the tag)


image_sharper:
Code to change the brightness and contrast of an image collected from any camera that publishes a sensor_msgs/Image ROS message and pass the edited image to the ar_track_alvar node which is started in the launch file. Camera can be specified in the launch file.

Usage:

    roslaunch image_sharper image_sharper.launch
