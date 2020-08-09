# ros_first_steps

ros_first_steps is a ROS package with the aim of creating a node capable of interacting with turtlesim and making drawings of figures through coordinates obtained through files.


## Installation
This package works on [Kinetic](http://wiki.ros.org/kinetic/Installation) version of ROS.



### ROS installation
First of all you need to have installed this distribution.

[Here](http://wiki.ros.org/kinetic/Installation/Ubuntu) is explained the instructions to install ROS kinetic on Ubuntu 15.10 and Ubuntu 16.04 if you don't have already installed it.

Once you **completed** the previous tutorial you can continue

Do not forget to source your installation path so you can have acces to the ROS commands.
```bash
$ source /opt/ros/kinetic/setup.bash
```
You will need to run this command on every new shell you open to have access to the ROS commands, unless you add this line to your .bashrc

#### Worskpace
If you already have a worskpace created jump to the next step.

Create your catkin workspace:
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

This will generate serveral files. You will need to source another file too.
```bash
$ source devel/setup.bash
```
As said before, if you don't want to run this command on every shell you open you should add this line to your .bashrc.

For more details about the creation of a workspace you could check [Create a ROS workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) page.

### Packages needed

This package uses both the rqt and turtlesim packages. Please install both packages, if you have not yet done so.

```bash
$ sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-turtlesim
```


### Install ros_first_steps package

To install this package and use it you have two options to do that. You can either download the package and paste all the files in $YOUR_WORSKPACE/src/ros_first_steps or you can git clone the repository from the /src path.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ekumenlabs/ros_first_steps
```

Finally you should run a make to generate the necessary files

```bash
$ cd ~/catkin_ws
$ catkin_make
```


## Usage

There is a fully configured ROS Launch file so that when executing it, the necessary nodes are started.

It executes:
- roscore
- turtlesim_node
- move_turtle_node (ros_first_steps's node)
- rqt_reconfigre

Speed is a parameter and rqt_reconfigure is exectued because you need this to dynamically change turtle's speed.

Inside the figures folder there are 3 files that correspond to 3 different figures. You can add more files to draw other figures.

To start, you shall execute the following line:


```bash
$ roslaunch ros_first_steps movingturtle.launch
```


This command line will start all the files needed and draw a five-point star.

<p align="center">
  <img src="https://i.imgur.com/PvduH8j.png" width="350" height="380">
</p>

If you want to draw another figure you have to specify the path of that file. It will draw
the figure located in ros_first_steps/figures/figure1.yaml if you do not specify a path.

```bash
$ roslaunch ros_first_steps movingturtle.launch figure:=<PATH>
```
This will draw the figure described from the coordenates within that file.

For example:
```bash
$ roslaunch ros_first_steps movingturtle.launch figure:=/home/$USER/catkin_ws/src/ros_first_steps/figures/figure2.yaml
```
This will draw the figure located in figure2.yaml.

Three figures are shared as examples. These are found within /ros_first_steps/figures.

When the turtle finishes the drawing node will be terminated.

### Speed parameter

With the rqt window you can change the speed of the turtle. 

<p align="center">
  <img src="https://i.imgur.com/1FlolkZ.png" width="500" height="300">
</p>



### Play or Pause Service
The node runs a service for pausing or resuming the movemente of turtle.

This service is called /playpause and it take one argument:
```bash
$ rosservice call /playpause pause
```
For pausing the movement.
```bash
$ rosservice call /playpause play
```
For resuming the movement and complete the remaiging path.

### Add a coordinates file
If you want to draw another thing you always can make a file following this rules:
- YAML format
- Group of coordinates named as points
- Max value of X or Y is 10
- Min value of X or Y is 0

Example:
```yaml
points:
-
  x: '6.5'
  y: '7.0'
-
  x: '10.5'
  y: '7.0'
-
  x: '7.5'
  y: '5.0'
-
  x: '8.5'
  y: '1.0'
-
  x: '5.5'
  y: '3.8'

```
It is neccesary to use a file extension such as .yaml

[Here](https://sourceforge.net/projects/graphixy/) is an useful program where you can draw something and convert it to coordinate.
Note: It ONLY works on Windows SO.


And if you need to transform your points from csv to yaml [here is website](https://onlineyamltools.com/convert-csv-to-yaml) 
where you can do it online:


## Roadmap
The idea of this project is to continue with the following statements:

- Implement a pid controller for the turtle, to improve the accuracy of the turtle when drawing at high speeds.
- Add in the yaml file the possibility to change the background color or the color and thickness of the pen. (It's almost complete)
- Add that you can draw in a window the route you want to do and that the turtle copy it.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License