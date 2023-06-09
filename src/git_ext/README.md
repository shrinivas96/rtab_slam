This folder only contains things that are external sources and are mostly here to try them out. Current contents:

1. scan tools. this is an attemmpt to use Andrea Censi's CSM. But the original one is not maintained by him anymore, and this seems to be someone else's ros wrapper for that? Did not build completly,. I did not look any further to make it work. The source is https://github.com/CCNYRoboticsLab/scan_tools.git with the branch being used: ros1
You can also directly install ros-noetic-laser-scan-matcher. More information https://wiki.ros.org/laser_scan_matcher. Though the github page asks you to install ros-noetic-scan-tools
Another git repo that uses the LSM from this repo and fixes some of the problems is given by https://github.com/nkuwenjian/laser_scan_matcher


2. Next to be tried: https://wiki.ros.org/crsm_slam Critical Rays Scan Matching. Uses scan-to-map matching instead of s2s.


3. https://laempy.github.io/pyoints/tutorials/icp.html is a set of ICP and normals ICP (from Grisetti et al.) implementation. The paper is here: https://ieeexplore.ieee.org/abstract/document/7353455

4.