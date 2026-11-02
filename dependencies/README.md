# Dependency status

The first implementation builds only the motion foundation, interfaces, mock runtime,
simulated supervisor, and bringup. It does not import FAST-LIO2, RTAB-Map, Nav2,
Scan Context, or Unitree SDK into the build yet. No empty dependency manifest is
presented as an integrated navigation system.

Baseline: Ubuntu 24.04, ROS 2 Jazzy, C++17 and Python 3.12. The interactive installer
pins the ROS apt-source bootstrap to 1.3.0 with the publisher's SHA-256 digest.
Individual apt package versions are recorded by the deployment environment and
are not yet a reproducible binary lockfile. CI checks use the official
`ros:jazzy-ros-base` image; this tag can receive upstream updates.

Upcoming algorithm/SDK integration must record exact upstream repositories and
commits, message compatibility, architecture and build evidence before those
dependencies are enabled. Installing the optional navigation dependencies is not
equivalent to integrating or validating them.
