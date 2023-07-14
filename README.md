# Particle_Swarm
This is the official code repository for the project Particle Swarm.
We provide instructions to setup a [docker container](#docker-setup) and [source build](#source-setup) to run an experiment. 

# Citation
When using the code from this repository, consider citing the associated paper: 

```
@article{varadharajan2022hierarchical,
  title={Hierarchical control of smart particle swarms},
  author={Varadharajan, Vivek Shankar and Dyanatkar, Sepand and Beltrame, Giovanni},
  journal={arXiv preprint arXiv:2204.07195},
  year={2022}
}
```

# Brief Approach 
We propose a method for the control of robot swarms using two subsets of robots: a large group of simple, oblivious robots (which we call the workers) that is governed by simple local attraction forces, and a smaller group (the guides) with sufficient mission knowledge to create and displace a desired worker formation by operating on the local forces of the workers. The guides coordinate to shape the workers like smart particles by changing their interaction parameters.
 

# States in Guide Swarm 
<table>
  <tr>
    <td><font size="5">Separation</td>
    <td><font size="5">Edge Following</td>
    <td><font size="5">Shaping Setup</td>
  </tr>
  <tr>
    <td><img src="Images/separation.gif" width=400 ></td>
    <td><img src="Images/edge_follow.gif" width=400></td>
    <td><img src="Images/shaping_setup.gif" width=400></td>
  </tr>
 </table>
<table>
  <tr>
    <td><font size="5">Shaping</td>
     <td><font size="5">Movement</td>
  </tr>
  <tr>
    <td><img src="Images/shaping.gif" width=400></td>
    <td><img src="Images/movement.gif" width=400></td>
  </tr>
 </table>

# Control Parameters of Worker Swarm

<table>
  <tr>
    <td><font size="5">Control Parameters</td>
    <td><font size="5">Configuring Density</td>
  </tr>
  <tr>
    <td><img src="Images/cp.png" width=1080></td>
    <td><img src="Images/CP_change.gif" width=400></td>
  </tr>
</table>

# Robot Behavior scripts
Robot behavior scripts were developed with [Buzz](https://the.swarming.buzz/) (an extensible programming language for robot swarms). Buzz provides many primitives for programming robot behaviors. We use some of these primitives, like virtual stigmergy, to propagate control parameters across the workers.
The simulation experiments were performed using the [ARGoS3](https://github.com/ilpincy/argos3) simulator (including the simulations corresponding to the moving plots above) and the [Khepera-IV plugin](https://github.com/ilpincy/argos3-kheperaiv) for ARGoS3.

# Repository organization 
```
+-- README.md
+-- Images (Images used in readme)
+-- Simulation (Simulation Experiment scripts)
|   +-- Docker_files
|       +-- Dockerfile
|       +-- entrypoint.sh
|       +-- run_docker.sh
|   +-- Convergence_experiments (Convergence of Worker control parameters experiments, fig.5)
|       +-- batch_scripts (scripts used for batch experimental runs on HPC clusters)
|       +-- buzz_script (buzz script used for the convergence experiment)
|       +-- Data_analysis (Notebooks used for data processing and plot generation)
|       +-- experiments (argos experiment files used for the experiment)
|       +-- loop_function (argos loop function used for experiment configuration and data logging)
|   +-- Movement_experiments (Shape formation and movement experimental files)
|       +-- batch_scripts (scripts used for batch experimental runs on HPC clusters)
|           +-- launch_job.sh (script to create jobs for the whole experimental set to be run on HPC)
|           +-- run_job.sh (script to run a single configuration)
|       +-- buzz_scripts (scripts used for the movement experiments)
|           +-- Object_movement_test.bzz (Unified script containing the robot behavior for both guides and workers)
|       +-- data_processing (Notebooks used for data processing and plot generation)
|       +-- experiments (experimental argos files and simulation configuration)
|           +-- template.argos (template argos file used by batch script like run_job.sh)
|           +-- template_exp.argos (an experimental instance that can run locally, use this to start the experiment)
|       +-- loop_funcs (argos loop function used for experiment configuration, robot placement and data logging)
|       +-- algorithmPseudocode.txt (Contains the pseudo code of the overall behavior)
|       +-- edge_following.txt (Contains the pseudo code of the edge following behavior)
```

# Setting up the simulation 
The following instructions assume the user is running a Debian OS (e.g. Ubuntu). 
The repository was tested on Ubuntu 20

## Docker Setup
Set up x11docker to interact with argos3 from the docker container, with the instruction from https://github.com/mviereck/x11docker. 

Building the docker container

```
$ git clone https://github.com/MISTLab/Particle_swarm.git
$ cd Simulation/Docker_files
$ docker build . --tag vivekshankarv/particle_swarm
```
Or you could pull from docker hub

```
docker pull vivekshankarv/particle_swarm
```

Testing the simulation

```
$ cd Simulation/Docker_files
$ xhost +local:
$ ./run_docker.sh
$ cd /home/docker/Hir/KheperaIV/particle_swarm/Simulation/Movement_experiments/experiments
$ argos3 -c template_exp.argos
```

Additional terminal console to the running docker container can be attached using

```
$ docker exec -it $(docker container ls -q) /bin/bash
```

## Source Setup

### Dependencies 
ARGoS3 (Multi-robot simulator)
Buzz (Programming language for Robot swarms)
KheperaIV plugin for ARGoS3

Follow the following steps to set up the dependencies (All the following instructions can be found in the respective repositories).

### Setting up ARGoS3 
One could choose to install ARGoS3 from binaries, and we provide the instructions to install it from the source.
```
# Install dependencies for building ARGoS3

$ sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev \
  lua5.3 doxygen graphviz libgraphviz-dev asciidoc

# clone argos3 

$ git clone https://github.com/ilpincy/argos3.git

# Build and install argos

$ cd argos3
$ mkdir build_simulator
$ cd build_simulator
$ cmake -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DARGOS_BUILD_FOR=simulator \
        -DARGOS_BUILD_NATIVE=OFF \
        -DARGOS_THREADSAFE_LOG=ON \
        -DARGOS_DYNAMIC_LOADING=ON \
        -DARGOS_USE_DOUBLE=ON \
        -DARGOS_DOCUMENTATION=OFF \
        -DARGOS_INSTALL_LDSOCONF=ON \
        ../src
$ sudo make install

```

### Setting up Buzz

```
# Clone Buzz source

$ git clone https://github.com/buzz-lang/Buzz.git buzz

# build and install Buzz

$ cd buzz
$ mkdir build && cd build
$ cmake ../src
$ make
$ sudo make install

# Update shared library links

$ sudo ldconfig
```

### Setting up KheperaIV plugin for ARGoS

```
# Clone KheperaIV plugin

$ git clone https://github.com/ilpincy/argos3-kheperaiv.git

# build and install

$ mkdir build_sim
$ cd build_sim
$ cmake -DCMAKE_BUILD_TYPE=Release ../src
$ make
$ sudo make install
```

### Setting up the Loop function for the particle swarm experiment

```
$ cd Simulation/Movement_experiments/loop_funcs
$ mkdir build
$ cd build
$ cmake ..
$ make
```

You are all set now and can try the particle swarm code.

## Testing 

The following launches an instance of the particle swarm experiment.
One could change the configuration in the loop function tab of the template_exp.argos to change the experimental configuration (e.g., a different shape or robot number).

```
$ cd Simulation/Movement_experiments/experiments
$ argos3 -c template_exp.argos
```

## Available configurations for the simulator. 

### Number of worker robots and guide robots. 

The worker numbers and guide numbers are coupled configurations. For instance, a worker number 50 should be accompanied by a guide number of 4.

```
sheep = {50 100 300 1000}
dog   = {4 8 16 32}
```

### Shapes

There are four available shapes. 
To create a custom shape, modify the function `load_targets_from_loop_fun` in the file `Simulation/Movement_experiments/buzz_scripts/includes/targets.bzz` to have the configuration for your custom shape. 

```
shape = {clover dumbbell none torus}
```
Note: none shape creates a circle shape.

### Movement

There are three available movements for a motion in the desired shape. 
To add a custom motion, modify the function `load_targets_from_loop_fun` in the file `Simulation/Movement_experiments/buzz_scripts/includes/targets.bzz`, along with the loop function to be able to support the string to integer conversion for movement enumeration.

```
movement = {straight diagonal rotation}
```

## Design parameters used in the simulations


|Parameter|value|
|---|---|
|k|0.2|
|$a_0$|0|
|a|$\frac{\sqrt{(0.9155 * N) / 0.1}}{ 2}+4$|
|b| -a|
|$d_{\rho}$ | 0.8 m|
|$d_{\delta}$ | 0.45 m|
|$\theta_{\delta}$ | 0.01 rad|
|$FOV$ | 1 m|
|$RFOV$ | 0.7 m|
|$d^e$ | 95|
