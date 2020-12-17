# Docker

This directory provides the user with a method to quickly build and test the
examples in a docker container on a Linux PC.  Scripts are provided to
create the docker image and to start, stop and attach to the docker container.

## Basic operation

The script `./build.bash` creates the docker container.  Do this
just once!  This process took a few minutes on my PC, so get on with
something else while the image is built.

To start the container, use `./start.bash`.  This script starts the
container and leaves it running until `./stop.bash` is called. The script
`./start.bash` also mounts an external directory on to the container
directory `/home/build/ws`.  The external directory can be specified when
calling `./start.bash`, e.g.

```bash
cd ~/my-workspace
~/git/ros2-examples/docker/start.bash `pwd`
```

If no path is given on the command line, the `start.bash` script will create
a workspace directory using the pattern `$HOME/<docker container name>_ws`.

If you need to change the mounted directory on the container, you need to
destroy the current container by stopping the container using `./stop.bash`,
remove the container using `./rm_container.bash` and then start a new
container using `.start.bash`.

When the container is running, you can get a Bash user prompt attached to the
container using `./attach.bash`.  The `./attach.bash` script can
be used to open multiple terminal sessions on the docker container by calling
the script as many times as needed.

## Setting up the workspace for the first time

The script `setup_examples.bash` sets up links from the code directory to the
code directory.  To run, attach a terminal shell to the running docker and
then run:

```bash
cd ~/ws
./setup_examples.bash
```

After that, you should be able to run all the examples using the supplied
launch files.
