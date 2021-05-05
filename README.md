# superMarketRobot
## Configure the environment
> Reference: 
> 
> [1] [webots_ros](http://wiki.ros.org/webots_ros)
> 
> [2] [Sample Simulations](http://wiki.ros.org/webots_ros/Tutorials/Sample%20Simulations)
```bash
sudo apt-get install ros-melodic-webots-ros
```
```bash
mkdir -p ~/superMarketRobot_ws/src
cd ~/superMarketRobot_ws/src
catkin_init_workspace
git clone -b melodic https://github.com/cyberbotics/webots_ros.git
```
```
cd ~/superMarketRobot_ws/
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic
catkin_make
source devel/setup.bash
```
## How to use ROS in webots
[Webots User Guide](https://cyberbotics.com/doc/guide/tutorial-8-using-ros?tab-ros=melodic)
[Nodes and API Functions](https://cyberbotics.com/doc/reference/nodes-and-api-functions?tab-language=ros)

## Test
### Move Test
```bash
roscore
webots # open webots world
rosrun webots_ros my_youbot
# Test move_base
rostopic pub -r 10 /youbot/move webots_ros/move "{moveType: 'Turn left', speed: 1.0, x: 0.0, z: 0.0, alphe: 0.0}" # can change moveType and other params
# Test arm
rostopic pub -r 10 /youbot/arm webots_ros/arm "type: ''
grabFlag: false
operateFlag: [false, false, false, false, false]
angle: [0.0, 0.0, 0.0, 0.0, 0.0]"
# Test kinect
rosrun webots_ros supermarket_robot

```
