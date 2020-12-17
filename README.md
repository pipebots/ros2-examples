# ROS2 Examples

These examples have been copied from another project, the Leeds Pump, that
used a message, a service and an action.  The Leeds Pump was a ROS2 wrapper
that sent commands to and received status updates from an Arduino that
used:

* A ROS2 service to control a gimbal to aim the pump nozzle.
* A ROS action to control the pump.
* A ROS2 message to publish status information including the amount of water
remaining in the tank.

I have stripped out most of the pump specific code and replaced it with ROS2
logging calls to make it more obvious how it all fits together.

The following demos have been implemented:

NONE

TODO:

* A server offering a status message publisher with a client with a message subscriber.
* A server offering a single service with a client that uses that service.
* A server offering a single action with a client that uses that action.
* Combined server that has a message publisher, service server and
action server with a client that uses all three interfaces.

