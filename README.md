# superMarketRobot
## Configure the environment
> Reference: 
> [1] [webots_ros](http://wiki.ros.org/webots_ros)
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

