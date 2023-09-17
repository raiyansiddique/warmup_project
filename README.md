# Warmup Project
### By Satchel and Raiyan

##  Teleop

For teleop we were given the task to create a robot controller using keyboard input. To solve this problem we broke down the robot movement into 5 different states: forward, backward, rotate right, rotate left and stop. By publishing to the `cmd_vel` topic, we can have the robot move in all 5 states, which is where we get the `forward`, `backward`, `right`, `left` and `stop` function. To connect these functions to the keyboard we use a while loop in the main function that waits for `w` to move forward, `s` to move backwards, ...

## Drive Square

For the drive square task, we are supposed to make the Neato drive in a square. To solve this task, we broke the square movement down into two main components rotating left 90 degrees and moving forward 1 meter. Using the `cmd_vel` topic, we first have the robot move forward 1 meter, then rotate 90 degrees and repeat this process until a square is created. The only issue we encountered is that the angular velocity is not one to one. We set the angular velocity to pi/4 radians per second, but had to use a scaling factor to get the rotation closer to 90 degrees. 

## Wall Following

For the wall following task, we are supposed to have NEATOs movement be parallel to a wall. To keep the task simpler, we have it follow a wall to the left of the robot. We subscribe to the scan topic to get the lidar scan data from the neato. Then we take the measurement at 90 degrees to see the distance the neato is from the wall. If it is within the range of 0.1 to 0.15 meters away from the wall then it uses the 45 degree and 135 degree measurement to follow the wall. If it is not within that range then the robot tries to get back into the range of that wall by rotating the direction it needs to while moving forward at the same time. 

The wall following logic is fairly simple. If both the 45 and 135 degree measurements are non zero, we subtract them. If the difference is less than 0.05 meters we know that the front of the neato is closer to the wall the back, so the robot needs to rotate right and move forward. If the opposite is true then the robot needs to rotate left and forward. 

## Person Follower 

For the person follower problem, we are tasked 