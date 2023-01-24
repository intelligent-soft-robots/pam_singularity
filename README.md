# PAM Apptainer/Singularity Images

[Apptainer](https://apptainer.org) definition files related to the PAM robot.


## Get Pre-built Images

You can pull pre-build versions of the `pam_base` and `pam_mujoco` images
directly with Apptainer. For example for the latest base image:

    $ apptainer pull oras://ghcr.io/intelligent-soft-robots/pam_singularity/pam_base:latest


## Images in this Repository

### `pam_base`

This image, contains all required dependencies to build the `PAM_MUJOCO` treep
project.

To build the workspace and run things inside the container do

    $ cd path/to/your_workspace
    $ apptainer shell --nv path/to/pam_base.sif
    Apptainer> colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=Release '

    Apptainer> . install/setup.bash  # source the workspace
    Apptainer> ros2 run <package_name> <executable_name>


### `pam_mujoco`

Extends the base image and installs the `PAM_MUJOCO` project inside the image.

It includes a file `/setup.bash` which needs to be sourced to set up
the environment for building/running the code.

Example:

    $ apptainer shell --nv path/to/pam_mujoco.sif
    Apptainer> source /setup.bash
    Apptainer> ros2 run <package_name> <executable_name>


### `learning_table_tennis_from_scratch`

Build with:

    make learning_table_tennis_from_scratch.sif

Adds the `learning_table_tennis_from_scratch` package to the `pam_mujoco` image.


## Manually Build Images

### Requirements

For building the base image only a recent version of
[Apptainer](https://apptainer.org) and an internet connection is required.

For building the images containing the PAM software the following applications
are required in addition:

- git
- [treep](https://pypi.org/project/treep/)


### General: Building Images

To build a specific image (let's say `pam_base.sif`), simply call:

    make pam_base.sif

Note that for some images, `treep` is used to get the project software.

To clean up run

    make clean

This deletes the `build` directory as well as built sif files.
