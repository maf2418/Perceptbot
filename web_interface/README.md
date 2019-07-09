
# Aim of Perceptbot web project:
Create a web based controller for the robot, useful for presentation purposes. 
Main html pages are:
- page_rviz.html
- page_info.html
- page_tf.html

The live video stream from the camera on the robot and the joystick to control the robot
    are contained in iframes on these pages so they have their own html files. 


## Hardware
- Raspberry Pi, ROS Kinetic Kame
    * roscore
    * rosnodes

- PC, Linux
    * gmapping
    * rosbridge
    * nginx webserver ??

more information can be found about assembly instructions in the folder: ...


## Configure IP addresses
The Pi IP: 192.168.1.67 needs configuring in the code to get rid of websocket connection errors:
    webui.js, 
    _IP_Robot.js,
    page_rviz.html

In the code we refer to the PC IP with localhost, if you want to run the website from another device this may have to be changed.
If you are running gmapping on the PC, rosbridge connection to the localhost


## Setup PC
- sudo apt install ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx

Configure local webserver, Nginx.
- sudo /etc/init.d/nginx stop
- /etc/nginx/sites-available/default 
    Replace: root /var/www/html; 
    With:    root /home/.../.../your_website_folder;
- sudo /etc/init.d/nginx start 
or sudo /etc/init.d/nginx restart

To make sure ROS master IP is set
- export ROS_MASTER_URI="http://192.168.1.67:11311" 

> Run ros nodes
- ./launch.sh

    To Activate explore node through website
        After launching all the nodes, the state machine python script can be activated on the PC.
        When the state machine node is listening, the top left button on the website 
        can toggle autonomous exploration by publishing a message via Rosbridge. 
 
> Run state machine
- cd state_machine/src
- python perceptbot_state_machine.py

Check camera
- http://localhost:8080/stream_viewer?topic=/usb_cam/image_raw

Check joystick, assuming nginx is running
- http://localhost/joystick_n.html
    
    Make sure the following nodes have been launched by launch.sh: 
    web_video_server, rosbridge_websocket, usb_cam_node
    
    They can also be launch individually:
        - roscore >/dev/null 2>&1 &
        - rosrun web_video_server web_video_server >/dev/null 2>&1 &
        - roslaunch rosbridge_server rosbridge_websocket.launch >/dev/null 2>&1 &
        - rosrun usb_cam usb_cam_node pixel_format:=yuyv >/dev/null 2>&1 &


## Setup Pi
- navigate to overlay_ws
- git clone ...
- catkin make in overlay_ws

Check ports
- roscore port: 11311
- rosbridge port: 9090
- nginx port: 80
- video_web_server port: 8080

> Run
- ./main.sh
or
- roscore
- roslaunch rosbridge_server rosbridge_websocket.launch
- rosrun usb_cam usb_cam_node pixel_format:=yuyv
- rosrun web_video_server web_video_server


## Useful commands:
Find out IP address
- ifconfig

Check if something is installed:
- dpkg -s <package-name> | grep Status

See port connections
- netstat -nl4t

See published commands
- rostopic echo /cmd_vel

Topics published
- rostopic list


## Software references:
    - https://medium.com/husarion-blog/bootstrap-4-ros-creating-a-web-ui-for-your-robot-9a77a8e373f9
    - http://robotwebtools.org/tools.html 
    - http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality 
    - https://github.com/RobotWebTools/roslibjs/blob/a00fc0d8fc4b0da7bca11a67d1c92cedcd16a5fc/src/core/Ros.js#L195-L213 
    - http://iguanatronics.com/simple-tutorial-on-rosbridge-and-roslibjs/ 
    - https://github.com/GT-RAIL/keyboardteleopjs 
    - http://wiki.ros.org/web_video_server#Available_URLs
    - http://wiki.ros.org/nav2djs/Tutorials/CreatingABasicNav2DWidget

