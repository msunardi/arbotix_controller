# arbotix_to_pololu

Requirements:
- ROS Arbotix package (http://wiki.ros.org/arbotix)
- ROS Arbotix Python package (http://wiki.ros.org/arbotix_python)
- Arbotix-M board
- HROS-1 (Jimmy) robot kit
- Pololu Maestro board + regular RC servos (Mini Maestro 18-channel was used in testing)

To use:

1. Set up ros_pololu_servo
   - Clone ros_pololu_servo (https://github.com/geni-lab/ros_pololu_servo) as one package in a catkin workspace
   - Build using `catkin_make`
   - Copy launch/jeeves_motors.yaml to ros_pololu_servo/launch/jeeves_motors.yaml
   - Make sure the servo max/min limits are correct in the [.yaml file](https://github.com/msunardi/arbotix_to_pololu/blob/master/launch/jeeves_motors.yaml)
   - Edit ros_pololu_servo/launch/pololu_example.launch to use jeeves_motors.yaml
   - Make sure the port name/number is correct
   - Launch pololu node: `roslaunch ros_pololu_servo/launch/pololu_exmple.launch`
2. Set up Arbotix-M board
   - Make sure to have arbotix_ros firmware installed on the Arbotix-M board
   - Use the `arbotix_terminal` tool that comes with the ROS Arbotix package to test
   - For more details, power supply, connections, see: http://learn.trossenrobotics.com/arbotix.html
3. Decide which servos on the Jimmy robot to be mapped to the servos controlled by the Pololu Maestro board
   - For example, see https://github.com/msunardi/arbotix_to_pololu/blob/master/src/jimmy_train_node.py#L62
4. Launch the train mode: `roslaunch launch/arbotix_trainer.launch`
5. Start moving the servos on the Jimmy robot and see the servos connected to the Pololu Maestro moves accordingly

Working example:
https://www.youtube.com/watch?v=W3DMSCJJ6dw
