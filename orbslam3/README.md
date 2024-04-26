# ORBSLAMv3

OrbSlam implementation with a Dockerfile and dependencies for easy deployment

## Installation

Simply build the docker image using devcontainer. This will give access to a terminal with a preinstalled orbslam package, and a set of test images.

If you want to run the map viewer in Linux, ensure that you run `xhost +local:docker` before running the docker or devcontainer. 

To turn off the visulalization of the trajectory/map created by ORB-SLAM, you can modify line 83 of mono_euroc.cc (or the equivalent line in another example script) to have a fourth flag of "false" rather than "true".
In other words:
`ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);`
`ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, false);` 

## Test run
Go to the built ORB-SLAM folder (`cd /ORB-SLAM`) and run
`./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ./MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt`



