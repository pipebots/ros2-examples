# ROS2 Examples

This repo contains code that is intended to show a developer how to implement
a ROS2 publisher, subscriber, service and action.  It is expected that the
code will be used for reference and copied into other projects.

These examples have been copied from another project, the Leeds Pump, that
used a message, a service and an action.  The Leeds Pump was a ROS2 wrapper
that sent commands to and received status updates over a serial port connected
to Arduino.  The Leeds Pump implemented:

* A ROS2 service to control a gimbal to aim the pump nozzle.
* A ROS action to control the pump.
* A ROS2 message to publish status information including the amount of water
remaining in the tank.

I have removed the pump specific code and replaced it with ROS2 logging calls
to make it more obvious how it all fits together.

The
[Docker README file](docker/README.md)
has details on how to build and run this project.

## Status

The following demos have been implemented:

* A server offering a status message publisher with a client that subscribes
to the publisher.
* A server offering a single service with a client that uses that service.
* A server offering a single action with a client that uses that action.

All demos can be run using launch files.

Implemented but broken

* Combined server that offers a message publisher, service server and
action server with a client that uses all three interfaces.
