# RT2 Assignment 2

*Francesco Ganci* - 4143910 - 2021/2022

> Compatible With:
> - ROS1 noetic

![cover](/docs/img/cover-jup.png)

## What is this?

This is another update of the project in this repository:  [programmatoroSeduto/rt2_assignment1 on GitHub](https://github.com/programmatoroSeduto/rt2_assignment_1/tree/action), branch *action*. The main drawback of the previous version of the project lies in the complete lack of a *user interface* allowing to make the user aware of the status of the robot, as well as to drive the robot. 

Here a user interface is provided using Jupyter and MatPlotLib. Compared to the original project, the code has been some little updates: the most of the work has been done on the Jupyter notebooks. 

### Differences from the previous version

In order to support the implementation on Jupyter, some things have been changed:

- no changes in `position_service.cpp`

- created a new message type, `JupyterTargetInfo.msg`, containing a large set of informations about the status of the node `state_machine.cpp`
	
- the node `state_machine.cpp` now publishes a message of type `JupyterTargetInfo` almost regularly, properly setting the fields depending on the state of the node. The message is employed by the graphical interface for the visuals. 

- the node `go_to_point.py` now publishes the command twist on two topics: `/cmd_vel` as before, and `/current_cmd_vel` read by the Jupyter notebook. This topic is needed for visualizing the commanc currently given to the robot in a graph. 

- the old node `user_interface.py` were replaced by the new implementation `jupyter_user_interface.py` providing a service for switching on/off the random behaviour. 

### Structure of the repository

This repository contains a package to install in a ROS1 workspace. Here is the main elements in the package:

```
/rt2_assignment_2
├── JupyterNotebooks
│   └── jupyter_interface_V2.ipynb	<> the user interface
│
├── launch
│   └── sim_jupyter.launch	<> launcher for Jupyter notebooks
│
├── msg
│   └── JupyterTargetInfo.msg	<> infos for the graphical interface
│
├── scripts
│   ├── go_to_point.py	<> motion planning algorithm (modified)
│   ├── jupyter_user_interface.py	<> an updated version able to communicate with Jupyter
│   └── user_interface.py	<> the old version
│
├── src
│   ├── position_service.cpp	<> random position generator
│   └── state_machine.cpp	<> implementation of the random behaviour
│
└── urdf
    └── my_robot.urdf			<> Gazebo robot description
```

# How to setup and run the project

Here are the instructions for installing and running the project in your ROS1 environment. 

## Prerequisites and Dependencies

Before installing the project, make sure your system satisfies these requirements:

- a working installation of ROS1 (the project is compatible with **ROS1 Noetic**).
	
	I suggest to use the Docker image here: [carms84/noetic_ros2 Docker image](https://hub.docker.com/r/carms84/noetic_ros2) forwarding also the port 8888 in order to have Juyter on Windows. 

- A workspace is needed for the installation of the package. 
	
	Here's a script for quickly creating a workspace:
	
	```bash
	#! /bin/bash

	cd /root
	mkdir test_ws
	cd test_ws
	mkdir src
	catkin_make
	cd src
	```

No external dependencies are required to run the project. 

### DEPT -- URDF robot description

The simulation uses Gazebo and the URDF model here: [CarmineD8/pioneer_ctrl on GitHub](https://github.com/CarmineD8/pioneer_ctrl). Downloading the model is not necessary to run the project: the model is already integrated with this package. 

### DEPT -- Jupyter Notebook and extensions

Installing Jupyter Notebook is needed for running the user interface along with the ROS1 package. Here are the script to install it. **Don't ignore the errors**: I did put here some notes about the most common errors; in case you ignore the errors during the installation, the program won't work. 

```bash
# before starting, make sure everything is updated/upgraded
# see troubleshooting if you're using the Docker image

# you need pip!
# sudo apt-get install pip
pip3 --help

# Jupyter is a web app, so a browser is needed
# sudo apt-get install firefox 
firefox --help

# Install Jupyter py modules
pip3 install traitlets
# chec traitlets version >= 5.1.1
pip3 install pygments==2.4.1
pip3 install pyyaml
pip3 install bqplot
pip3 install ipywidgets

# install Jupyter Notebook
pip3 install jupyter
```

The project also requires some extensions, so Jupyter should be enabled for using them. What you need now is `nbextensions`, a ... extension that enables to install other extensions (wooooooooooooooooooooooooow...):

```bash
pip3 install jupyter_contrib_nbextensions
pip3 install jupyter_nbextensions_configurator

jupyter contrib nbextension install
# the output from this command should be very long, but with no errors or warnings
```

Here are some extensions you should enable for running the project. Mainly two extensions. The frist one is `widgetsnbextension`:

```bash
jupyter nbextension enable --py widgetsnbextension

# the expected output is:
# $$ jupyter nbextension enable --py widgetsnbextension
#   Enabling notebook extension jupyter-js-widgets/extension...
#     - Validating: OK
```

Another required extension for this project is `jupyros` which contains many functionalities useful for making Jupyter able to interact with ROS. Here's the installation script:

```bash
source /opt/ros/noetic/setup.bash

# install jupyros
pip3 install jupyros

# add Jupyros to Jupyter Notebook
jupyter nbextension enable --py --sys-prefix jupyros

# expected output from the last command:
# $$ jupyter nbextension enable --py --sys-prefix jupyros
#    Enabling notebook extension jupyter-ros/extension...
#      - Validating: OK
```

Below is a script for launching Jupyter. It is dentical to the one in the repository (root folder of the pacakge. 

```bash
#! /bin/bash

cd /
# see https://github.com/Jupyter-contrib/jupyter_nbextensions_configurator/issues/72
jupyter nbextensions_configurator enable --user
jupyter nbextension enable --py --sys-prefix jupyros
jupyter notebook --allow-root --ip 0.0.0.0
```

## Installation

Here are the instructions to install the project. 
Let's suppose the workspace you're using to install the project is `test_ws`; here you don't need a specific path: the workspace can be everywhere you want. 

Follow these steps:

1. go inside the `/src` folder of your workspace

2. clone the repo:
	
	```bash
	git clone https://github.com/programmatoroSeduto/rt2_assignment_2.git -b main ./rt2_assignment_2
	```
	
3. build the workspace with `catkin_make`. 

## Run the project

As usual, two ways for running this architecture: the quick one, and the other one node by node.

### First way - quick run

Let's start both the simulation and the Jupyter Notebook containing the interface. Follow the steps below:

1. open a bash and run the ROS1 master: `roscore &`. *Remember to source your workspace!*
	
2. go inside the folder of the package and run Jupyter:
	
	```bash
	roscd rt2_assignment_2
	./run_jupyter.sh
	```
	
	If you're opening Jupyter for the first time, the best is to set a password, so the next time it will be easier to use it. 
	
3. in another console (even directly in Jupyter Notebook) launch the project with the launch file `sim_jupyter.launch`.	
	
	```bash
	roslaunch rt2_assignment_2 sim_jupyter.launch
	```

4. Now everything should be running. Open the notebook you find inside the package, named `jupyter_interface_V2.ipynb` (see the folder JupyterNotebooks). 
	
	I always use *Kernel -> Restart & Run All* to start all the blocks of the notebook.
	
Please refer to the infos inside the notebook. Have fun!

### Second way - node by node

The way to launch the project is quite similar to the one shown in the readme of the previous verson of the project (link here: [programmatoroSeduto/rt2_assignment1 on GitHub](https://github.com/programmatoroSeduto/rt2_assignment_1/tree/action)) but launching the node `jupyter_user_interface.py` instead of `user_interface.py`. 

# Troubleshooting

Here are some well-known problems that can occur trying to run the project. 

## Update/Upgrade ROS

Especially when you use the Docker image [carms84/noetic_ros2](https://hub.docker.com/r/carms84/noetic_ros2), the first time you launch the command `sudo apt-get update`, an error message is print; the output is similar to this one:

```
$$ sudo apt-get update
sudo: setrlimit(RLIMIT_CORE): Operation not permitted
Get:1 http://dl.google.com/linux/chrome/deb stable InRelease [1,811 B]
Get:2 http://mirrors.ubuntu.com/mirrors.txt Mirrorlist [142 B]                                                                                 
Get:3 http://giano.com.dist.unige.it/ubuntu focal InRelease [265 kB]                                                                 
Get:6 http://dl.google.com/linux/chrome/deb stable/main amd64 Packages [1,090 B]                                                                                     
Get:5 http://ubuntu.mirror.garr.it/ubuntu focal-backports InRelease [108 kB]                                                                                         
Get:7 http://security.ubuntu.com/ubuntu focal-security InRelease [114 kB]                                                                        
Get:4 http://giano.com.dist.unige.it/ubuntu focal-updates InRelease [114 kB]                                              
Get:9 http://ubuntu.mirror.garr.it/ubuntu focal-updates/main amd64 Packages [2,185 kB]                                               
Ign:8 http://ubuntu.connesi.it/ubuntu focal-updates/restricted amd64 Packages                                                             
Ign:8 http://giano.com.dist.unige.it/ubuntu focal-updates/restricted amd64 Packages                                                                            
Ign:11 http://ubuntu.connesi.it/ubuntu focal-updates/multiverse amd64 Packages                                                                                 
Ign:12 http://giano.com.dist.unige.it/ubuntu focal-backports/universe amd64 Packages                                                     
Get:13 http://ubuntu.mirror.garr.it/ubuntu focal-backports/main amd64 Packages [51.2 kB]                                                 
Get:14 http://packages.ros.org/ros/ubuntu focal InRelease [4,676 B]                                                                
Get:11 http://ubuntu.mirror.garr.it/ubuntu focal-updates/multiverse amd64 Packages [30.3 kB]                           
Err:14 http://packages.ros.org/ros/ubuntu focal InRelease                                              
  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
Get:10 http://archive.ubuntu.com/ubuntu focal-updates/universe amd64 Packages [1,153 kB]
Get:15 http://packages.ros.org/ros2/ubuntu focal InRelease [4,679 B]            
Get:16 http://security.ubuntu.com/ubuntu focal-security/restricted amd64 Packages [1,139 kB]  
Err:15 http://packages.ros.org/ros2/ubuntu focal InRelease            
  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
Get:17 http://security.ubuntu.com/ubuntu focal-security/multiverse amd64 Packages [25.8 kB]
Get:18 http://security.ubuntu.com/ubuntu focal-security/main amd64 Packages [1,771 kB]
Get:8 http://archive.ubuntu.com/ubuntu focal-updates/restricted amd64 Packages [1,214 kB]       
Get:12 http://archive.ubuntu.com/ubuntu focal-backports/universe amd64 Packages [26.0 kB]
Get:19 http://security.ubuntu.com/ubuntu focal-security/universe amd64 Packages [870 kB]      
Fetched 9,079 kB in 7s (1,331 kB/s)                                                                                                                                                                               
Reading package lists... Done
W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros/ubuntu focal InRelease: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros2/ubuntu focal InRelease: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: Failed to fetch http://packages.ros.org/ros/ubuntu/dists/focal/InRelease  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: Failed to fetch http://packages.ros.org/ros2/ubuntu/dists/focal/InRelease  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: Some index files failed to download. They have been ignored, or old ones used instead.
```

This happens because there are still some old addresses in the list of the repositories. To solve this, here are the commands:

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# now, update/upgrade
sudo apt-get update
sudo apt-get upgrade
```

If you're interested, here is the official post explaining this fact: [ROS GPG Key Expiration Incident on ROS Discourse](https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669). Another useful post: [apt update: signatures were invalid: F42ED6FBAB17C654 - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/379190/apt-update-signatures-were-invalid-f42ed6fbab17c654/)

## Jupyter Notebook Installation - errors during the installation of *pyyaml*

The command generating the error:

```bash
pip3 install pyyaml
```

it could happen that this kind of error is print:

```
ERROR: pandas 1.4.2 has requirement numpy>=1.18.5; platform_machine != "aarch64" and platform_machine != "arm64" and python_version < "3.10", but you'll have numpy 1.17.4 which is incompatible.
ERROR: nbconvert 6.5.0 has requirement jinja2>=3.0, but you'll have jinja2 2.10.3 which is incompatible.
```

This should solve the problem:

```bash
pip3 install numpy --upgrade
# check that the version is >=1.22.3

pip3 uninstall jinja2
pip3 install jinja2
# check that the version is >=3.1.1

# try again
pip3 install pyyaml
```

## catkin_make - Unable to find either executable 'empy' or Python module 'em'

There could be an error when also Anaconda is installed in the same system. The error procudes this output on the screen:

```
-- Found PythonInterp: /root/anaconda3/bin/python3 (found suitable version "3.9.7", minimum required is "3") 
-- Using PYTHON_EXECUTABLE: /root/anaconda3/bin/python3
-- Using Debian Python package layout
-- Could NOT find PY_em (missing: PY_EM) 
CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
  Unable to find either executable 'empy' or Python module 'em'...  try
  installing the package 'python3-empy'
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/all.cmake:164 (include)
  /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:20 (include)
  CMakeLists.txt:58 (find_package)


-- Configuring incomplete, errors occurred!
See also "/root/test_ws/build/CMakeFiles/CMakeOutput.log".
Invoking "cmake" failed
```

As workaround, you can call `catkin_make` as follows:

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

See [this post](https://answers.ros.org/question/353111/following-installation-instructions-catkin_make-generates-a-cmake-error/).

## Jupyter Notebook installation - strange errors from pip3

Sometimes there are some errors with `pip3` during the installation of the packages for Jupyter Notebook. The error could be this for example:

```
ERROR: ipython 8.2.0 has requirement pygments>=2.4.0, but you'll have pygments 2.3.1 which is incompatible.
ERROR: nbconvert 6.5.0 has requirement jinja2>=3.0, but you'll have jinja2 2.10.3 which is incompatible.
ERROR: nbconvert 6.5.0 has requirement pygments>=2.4.1, but you'll have pygments 2.3.1 which is incompatible.
ERROR: pandas 1.4.2 has requirement numpy>=1.18.5; platform_machine != "aarch64" and platform_machine != "arm64" and python_version < "3.10", but you'll have numpy 1.17.4 which is incompatible.
```

The error is solved in this post on [StackOverflow](https://stackoverflow.com/questions/69809832/ipykernel-jupyter-notebook-labs-cannot-import-name-filefind-from-traitlets). 

```bash
# try this
pip3 install jupyter bqplot pyyaml ipywidgets

# if something goes wrong, 
pip3 install traitlets==5.1.1
pip3 install pygments==2.4.1
pip3 install jupyter bqplot pyyaml ipywidgets

# the error should disappear.
```

## Jupyter notebook installation - jinja2

The packahe Jinja typically gives some problem; unfortunately it is required during the installation og Jupyter notebook, otherwise, as you try to run `jupyter contrib nbextension install`, it wll appear a very unintellegible Python error. 

In order to avoid this boring error, uninstall and install again this component:

```bash
pip3 uninstall Jinja2
pip3 install Jinja2
```

See this post, not directly related with this situation: [ContextualVersionConflict when starting 0-robot #139
 on HitHub](https://github.com/zero-os-no-longer-used/0-robot/issues/139)
 
 
## roscd - ROSCD NOT WORKING (package not found)

That's because you forgot to source the workspace, or maybe you didn't run `catkin_make` on it. So, `catkin_make` and `source` it before using `roscd`. 

# Author and Contacts

A project by *Francesco Ganci*, S4143910, upon a code by [CarmineD8](https://github.com/CarmineD8).

- **Email** : _s4143910@studenti.unige.it_

# See also

- [programmatoroSeduto/rt2_assignment1 on GitHub](https://github.com/programmatoroSeduto/rt2_assignment_1/tree/action)
- [CarmsD8/rt2_assignment1 on GitHub](https://github.com/CarmineD8/rt2_assignment1)
- [CarmineD8/pioneer_ctrl on GitHub](https://github.com/CarmineD8/pioneer_ctrl)
