# ROS2 Examples

This repo contains code that is intended to show a developer how to implement a ROS2 publisher, subscriber, service and action.  It is expected that the code will be used for reference and copied into other projects.

The [Docker README file](docker/README.md) has details on how to build and run this project in a docker container.

This repo shows how to setup a docker container and how to use some convenience scripts to make it easier to use with an example ROS2 project.

__NOTE: The code worked in 2021 but has not been tested since, so please use for inspiration!__

NOTE: Personal experience has shown that having custom ROS messages in a package with code, as in this package, does not scale well.  In a larger project with multiple repos with many custom ROS messages, it is better to put all project specific messages in a separate messages package.  This approach prevents "dependency hell" where you can't build one package because it depends on a message from another package that depends on a message from the first package.

## Background

These examples have been copied from another project, the Leeds Pump, that used a message, a service and an action.  The Leeds Pump was a ROS2 wrapper that sent commands to and received status updates over a serial port connected to Arduino.  The Leeds Pump implemented:

* A ROS2 service to control a gimbal to aim the pump nozzle.
* A ROS action to control the pump.
* A ROS2 message to publish status information including the amount of water remaining in the tank.

I have removed the pump specific code and replaced it with ROS2 logging calls to make it more obvious how it all fits together.

## Status

The following demos have been implemented:

* A server offering a status message publisher with a client that subscribes to the publisher.
* A server offering a single service with a client that uses that service.
* A server offering a single action with a client that uses that action.
* Combined server that offers a message publisher, service server and action server with a client that uses all three interfaces.

## Acknowledgments

This work is supported by the UK's Engineering and Physical Sciences Research Council (EPSRC) Programme Grant EP/S016813/1

© 2020,2021,2025 University of Leeds.

The author, A. Blight, has asserted their moral rights.
