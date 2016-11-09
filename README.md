# ros-gazebo-example
A navigation example using turtle bot to sweeping through an environmnet

# Installation 

## dependencies

We will be using the turtle bot as our working example robot. First you will have to install the ros packages 
for the turtle bot. These packages depend on gazebo2 which is the default provided when you install indigo-full-desktop.
If you want to use a more recent version of gazebo you will have to build the turtlebot packages from source.

	
```bash
sudo apt-get install ros-indigo-turtlebot
sudo apt-get install ros-indigo-turtlebot-*
```

Then run the following command:

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

If all goes well you should see the following:
<center>
<img src="docs/gazebo_screen_shot.png" width="400"/> 
</center>

If Gazebo loads up and the screen is black (it was the case for me) follow the fix instructions on this 
page: [gazebo2-with-black-screen](http://answers.gazebosim.org/question/12773/gazebo2-with-black-screen/).

## this package

In your catking_workspace/src

```bash
git clone https://github.com/gpldecha/turtlebot-navigation.git 
```
# Goal

This goal is to be able to sweep the square environment first with one robot and then 
collaboratively with another.

# Approach

We will be using the ros [navigation stack](http://wiki.ros.osuosl.org/navigation). The following concepts
will be important


* **global map**: is computed with SLAM and then stored as an occupancy grid map.

* [**local costmap**](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)(see section 2. The Costmap)
is used by a local navigation planner ensures that obstacles are avoided. The global path is projected onto the local costmap and is addapted according to the local surounding. See [base_local_planner](http://wiki.ros.org/base_local_planner?distro=kinetic) for an example of a local planner

* **global planners**: see [carrot_planne](http://wiki.ros.org/carrot_planner?distro=indigo) for an example.


## notes

There are two maps which are used during the execution of a plan

* 


# resources

### turtle bot

* [Setup the Navigation Stack for TurtleBot](http://wiki.ros.osuosl.org/turtlebot_navigation/Tutorials/indigo/Setup%20the%20Navigation%20Stack%20for%20TurtleBot)

* [Make a map and navigate with it
](http://wiki.ros.org/turtlebot_simulator/Tutorials/hydro/Make%20a%20map%20and%20navigate%20with%20it)

* [turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator)

* [learn turtle bot](http://learn.turtlebot.com/2015/02/03/3/)

### Drones in Gazebo

[gazebo drones](http://gazebosim.org/blog?page=2)

