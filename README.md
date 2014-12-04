# Introduction

This is a C++ implementation of OpenTLD that was originally published in MATLAB by Zdenek Kalal. OpenTLD is used for tracking objects in video streams. What makes this algorithm outstanding is that it does not make use of any training data. This implementation is based solely on open source libraries, meaning that you do not need any commercial products to compile or run it.

This version as PCL integration. Using a 3D camera, you can track a point in depth AND in position in the 2D image

Here is the list and description of all ROS parameters used in OpenTLD :
• queue size : int, default 100.
Set the queue size for the subscribers.
• Tracking3D : bool, default true.
Determine if OpenTLD needs only to do 2D tracking of the object
(false) or if a full 3D Tracking using the Point Cloud to send informa-
tion.
• Graphical interface : bool, default true.
Set the use of a graphical interface.
• ShowTrajectory : bool, default false.
If true, the trajectory of the object in the 2D image will be shown and
recorded.
• Trajectory length : int, default 50.
If ShowTrajectory is set to true : length of the trajectory shown, in
number of positions.
• TimeBetweenFrames : double, default 0.1.
Maximum time delay in between now and the frame used by openTLD
to have a good latency.
• LoadModel : bool, default false.
Set to true, OpenTLD will try to load the model situated at the path
provided by the next parameter, Path2Model.
• Path2Model : string, default ” /model”.
A path to a previously recorded model to be used by OpenTLD if
LoadModel is set to True
• SavingFile : string, default ” /model”.
A path where to save the model currently used by OpenTLD, if re-
quested by the user.

Here is a keyboard shortcuts : 
• l : toggle learning.
• a : toggle alternating mode (if true, detector is switched off when
tracker is available).
• e : export Model.
• i : import Model.
• c : clear model, reinitialize the tracking.
• r : ask the user for a new object in the current frame.
• n : ask the user for a image from the current frame to add in the
learning.
