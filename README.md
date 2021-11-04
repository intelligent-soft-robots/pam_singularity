# PAM Singularity Images

Singularity definition files related to the PAM robot.


## Requirements

For building the base image only a recent version of
[Singularity](https://singularity.hpcng.org) and an internet connection is
required.

For building the images containing the PAM software the following applications
are required in addition:

- git
- [treep](https://pypi.org/project/treep/)

Further you will need a GitHub account with an SSH key set up, otherwise cloning
the repositories via `treep` will not work.


## General: Building Images

To build a specific image (let's say `pam_base.sif`), simply call:

    make pam_base.sif

Note that for some images, `treep` is used to get the project software.  This
requires that a SSH key is set up for git and activated (e.g. via `ssh-add`).


To clean up run

    make clean

This deletes the `build` directory as well as built sif files.


## Images in this Repository

### `pam_base`

Build with:

    make pam_base.sif

This image, contains all required dependencies to build the `PAM_MUJOCO` treep
project.

To build the workspace and run things inside the container do

    $ cd path/to/your_workspace
    $ singularity shell --nv path/to/pam_base.sif
    Singularity> colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=Release '

    Singularity> . install/setup.bash  # source the workspace
    Singularity> ros2 run <package_name> <executable_name>


### `pam_mujoco`

Build with:

    make pam_mujoco.sif

Extends the base image and installs the `PAM_MUJOCO` project inside the image.

It includes a file `/setup.bash` which needs to be sourced to set up
the environment for building/running the code.

Example:

    $ singularity shell --nv path/to/pam_mujoco.sif
    Singularity> source /setup.bash
    Singularity> ros2 run <package_name> <executable_name>


### `learning_table_tennis_from_scratch`

Build with:

    make learning_table_tennis_from_scratch.sif

Adds the `learning_table_tennis_from_scratch` package to the `pam_mujoco` image.
