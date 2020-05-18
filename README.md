# Face-Tracking-Camera-Gimbal

Storm32 gimbal and camera are interfaced using ROS to the system with a usb cable where a centroid tracking algorithm is used to detect a face using a pre-trained caffe model and then track it. The algorithm has a face memory and can remember a face if it disappears from frame and reappears before timeout.

##Required Packages

[https://github.com/mcgill-robotics/ros-storm32-gimbal](ros-storm32-gimbal)

####References
[https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/](Pyimagesearch)
