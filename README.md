Team Members:
Zhenyuan Liang, Bruce Ping, Evan LaBanca, Elena Wang, Alexander Teng

In the src folder:
cybertruck2.py is the source code for part 1 (we set the parameters for green flag. To test part 1, please use green flag. If you have to use flag in other color, you should uncomment some code and tune the color range by yourself)
part2.py is the source code for part 2

In the launch folder:
drive.launch is the launch file for part 1
part2.launch is the launch file for part 2

In the script folder:
There are some pictures produced for capturing the image and pixel and for testing

To launch the source code and vesc: roslaunch drive.launch or roslaunch part2.launch in the right directory

Note that the launch file only launches the source code and vesc
To launch the camera, please open another terminal and type "roslaunch jetson_csi_cam jetson_csi_cam.launch width:=1280 height:=720 fps:=15"
The low resolution might help increase the detection speed of the robot.

Please do not try to simply include the xml code to launch the camera. It will not work even though TA tried this before. It might need further work if you want to achieve this. 

Project Tasks:
Part 1: Capture the Flag
Given an image of a colored sign, the robot must scan its surroundings to find a
similar sign, the flag, somewhere in the room. Once the flag has been detected,
the robot must move towards the flag to capture it. We suggest breaking this
task into these parts:
1. Create a function that saves the color from an image register_flag(img)
to a local file, flag.txt (5pts)
2. Next, write a callback function that will subscribe to images being streamed
from the camera topic. Recall that this topic is usually /csi_cam_0/image_raw.
(To jog your memory, take a look at this ROS tutorial on writing a Subscriber
node.) (20pts)
3. Create a function detect_flag() which takes the most recent image and
searches for the color (the values we stored in flag.txt). (15pts)
4. You will need to have your robot move around and explore the environment,
search_env(). To move the car, you should publish to the topic
mux/ackermann_cmd_mux/input/navigation. You can take a look at an example
implementation of publishing this at this file: mushr control (25pts)
5. Once the robot detects the flag, it should stop searching, and begin moving
towards the target. (10pts)
6. If the target moves at this point, the robot should be able to follow along.
(10pts)

Part 2: Racetrack
Now that your car is able to track an object, create a program that will move along a course with alternating color markers.
??? For this, you may demonstrate that your program works for a sequence of alternative colored markers in a room. You can test your code out with two different colored markers placed in the environment at about the height of the car.
??? Your program will then need to work with a track we will set up on the fifth floor. You may assume the next marker in the sequence is visible if the correct path is followed.
??? We will score based on the ability of the program to locate the alternating colors waypoints and follow the course.
??? You will also be graded by how quickly your car is able to complete our course. The team with the fastest performance will receive bonus points.

