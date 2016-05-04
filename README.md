# human_control_arm
use your arm to control a machine arm with the help of openNI in ubuntu.
You need to install ROS indigo, openNI, openCV firstly.

/*
*   You can use this to get an narrow of 2DOF arm move. Just for fun. use you arm to control the machine arm.
*   This is a demo of 2DOF's arm moving follow a human's arm(right or left)
* 
*   Please according to xtion's axes.
*   
*   Human's skeleton is got by the package which was wrote by Alessio with the help of `openni2_tracker`.
*   you can find it in `https://github.com/Chaos84/skeleton_tracker`
*
*/

/*
*   						Rule
*   Put your right hand in your right shoulder means to STOP.
*   
*   Put your right hand horizonally means to start. And the machine arm will follow your left arm's action.
*/

