# System messages

This package contains all of the .msg, .srv and .action file used for the
Leeds Pump.  Using a separate package makes building numerous packages much
easier as most packages are only dependent on the single msgs package instead
of all packages in the project.

Based on recommendations from this Q&A.
<https://answers.ros.org/question/288890/building-an-action-server-and-action-client-located-in-two-separate-packages/>
