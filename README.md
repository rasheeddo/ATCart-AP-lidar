This is a sample code to use with https://github.com/rasheeddo/testYDLidar

The purpose is to send MAVLink message OBSTACLE_DISTANCE back to CUBE pilot for object avoidance.

Please check on AP_object_avoidance.py

I copied just a necessary function from Thein Nguyen's code and adapt to my own, he is using Intel Realsense D435 for a distance sensor, but I am using YDLidar. So please go check his repo for more detail.

https://github.com/thien94/vision_to_mavros/blob/master/scripts/d4xx_to_mavlink.py
