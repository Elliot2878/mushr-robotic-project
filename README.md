In the src folder:<br>
cybertruck2.py is the source code for part 1 (we set the parameters for green flag. To test part 1, please use green flag. If you have to use flag in other color, you should tune the color range by yourself).<br>
part2.py is the source code for part 2<br>

In the launch folder:<br>
drive.launch is the launch file for part 1<br>
part2.launch is the launch file for part 2<br>

In the script folder:<br>
There are some pictures produced for capturing the image and pixel and for testing<br>
<br>
To launch the source code and vesc: roslaunch drive.launch or roslaunch part2.launch in the right directory
<br>
Note that the launch file only launches the source code and vesc<br>
To launch the camera, please open another terminal and type "roslaunch jetson_csi_cam jetson_csi_cam.launch width:=1280 height:=720 fps:=15"<br>
The low resolution might help increase the detection speed of the robot.<br>

Please do not try to simply include the xml code to launch the camera. It will not work even though TA tried this before. It might need further work if you want to achieve this.<br>

Project Tasks:<br>
Part 1: Capture the Flag<br>
Given an image of a colored sign, the robot must scan its surroundings to find a
similar sign, the flag, somewhere in the room. Once the flag has been detected,
the robot must move towards the flag to capture it. We suggest breaking this
task into these parts:
1. Create a function that saves the color from an image register_flag(img)
to a local file, flag.txt 
2. Next, write a callback function that will subscribe to images being streamed
from the camera topic. Recall that this topic is usually /csi_cam_0/image_raw.
3. Create a function detect_flag() which takes the most recent image and
searches for the color (the values we stored in flag.txt). 
4. Have robot move around and explore the environment,
search_env(). To move the car, publish to the topic
mux/ackermann_cmd_mux/input/navigation. Take a look at an example
implementation of publishing this at this file: mushr control 
5. Once the robot detects the flag, it should stop searching, and begin moving
towards the target. 
6. If the target moves at this point, the robot should be able to follow along.


Part 2: Racetrack<br>
Now that the car is able to track an object, create a program that will move along a course with alternating color flags.
* Applied PID controller algorithms on the robot to control the direction, velocity, and vibration
* Tested the robot that can move along a course with alternating color flags


