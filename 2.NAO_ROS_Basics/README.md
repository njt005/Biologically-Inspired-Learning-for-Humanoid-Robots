The goal of this mini-project was to learn learn how to work with the NAO robot in Webots simulation.  Multiple simple motion commands (position control) are sent to the robot.  Additionally, the received camera images from the NAO are processed.

Implementations:
1. NAO moves to a safe home position. When inputting the keyboard signal 'h' into the terminal, the NAO moves its arms into that home position. When inputting 'l' or 'r', the NAO does a repetitive waving motion with its left or right arm respectively.  Lastly, when inputting a 'b' signal, NAO's arms move simultaneously mirroring each other.

2. I familiarized myself with basic image processing using OpenCV with ROS:

https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
http://docs.opencv.org/doc/tutorials/tutorials.html
https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html

Then, I was able to process the raw images delivered by NAO's top camera to extract red color blobs. For this purpose, the image was transferred to HSV color space. The largest color blob was detected and the position of its center in pixel coordinates was calculated. This was then output via the terminal as well as shown in the camera image. The color on the back of the NAOs hand was used for the blob detector.
