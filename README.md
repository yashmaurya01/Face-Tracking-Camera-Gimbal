# Face-Tracking-Camera-Gimbal

Storm32 gimbal and camera are interfaced using ROS to the system with usb cables where a centroid tracking algorithm is used to detect a face using a pre-trained caffe model and then track it. The algorithm has a face memory and can remember a face if it disappears from the frame and reappears before timeout.The face tracker node sends the centroid coordinates to the gimbal_control node which sends the movements to the gimbal using the ros-storm-32-gimbal package codes

## Required Packages

[ros-storm32-gimbal](https://github.com/mcgill-robotics/ros-storm32-gimbal)

## References
[Pyimagesearch](https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/)
