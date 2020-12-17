# System messages

This package contains all of the .msg, .srv and .action file used by the 
drones.  Using a separate package makes building numerous packages much easier
as most packages are only dependent on the single msgs package instead of all
packages in the project.

Based on recommendations from this Q&A.
<https://answers.ros.org/question/288890/building-an-action-server-and-action-client-located-in-two-separate-packages/>

## Converting to ROS2

This was interesting as this was the first package I converted.  This package 
doesn't build anything so the package.xml file is different. This tutorial 
<https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/> was really 
useful when helping to convert the files. 

The messages, services and actions all stay the same. Only the CMakeLists.txt
and package files change. 
