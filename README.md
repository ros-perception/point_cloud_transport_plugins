# point_cloud_transport_plugins

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ROS2 Distro | Build Status | Package build |
:---------: | :----: | :----------: |
Rolling |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__point_cloud_transport_plugins__ubuntu_noble_amd64)](https://build.ros2.org/job/Rdev__point_cloud_transport_plugins__ubuntu_noble_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__point_cloud_transport_plugins__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__point_cloud_transport_plugins__ubuntu_noble_amd64__binary/) |
Jazzy |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__point_cloud_transport_plugins__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__point_cloud_transport_plugins__ubuntu_noble_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__point_cloud_transport_plugins__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__point_cloud_transport_plugins__ubuntu_noble_amd64__binary/) |
Humble |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__point_cloud_transport_plugins__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__point_cloud_transport_plugins__ubuntu_jammy_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__point_cloud_transport_plugins__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__point_cloud_transport_plugins__ubuntu_jammy_amd64__binary/) |

### draco_point_cloud_transport

ROS2 Distro | Package build |
:---------: | :----------: |
Rolling | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__draco_point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__draco_point_cloud_transport__ubuntu_noble_amd64__binary/) |
Jazzy | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__draco_point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__draco_point_cloud_transport__ubuntu_noble_amd64__binary/) |
Humble | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__draco_point_cloud_transport__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__draco_point_cloud_transport__ubuntu_jammy_amd64__binary/) |

### zlib_point_cloud_transport

ROS2 Distro | Package build |
:---------: | :----------: |
Rolling | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__zlib_point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__zlib_point_cloud_transport__ubuntu_noble_amd64__binary/) |
Jazzy | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__zlib_point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__zlib_point_cloud_transport__ubuntu_noble_amd64__binary/) |
Humble | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__zlib_point_cloud_transport__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__zlib_point_cloud_transport__ubuntu_jammy_amd64__binary/) |

### zstd_point_cloud_transport

ROS2 Distro | Package build |
:---------: | :----------: |
Rolling | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__zstd_point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__zstd_point_cloud_transport__ubuntu_noble_amd64__binary/) |
Jazzy | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__zstd_point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__zstd_point_cloud_transport__ubuntu_noble_amd64__binary/) |
Humble | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__zstd_point_cloud_transport__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__zstd_point_cloud_transport__ubuntu_jammy_amd64__binary/) |

This metapackage contains the most common plugins for pointcloud compression using [point_cloud_transport](https://wiki.ros.org/point_cloud_transport).

Currently provided are:

- [draco_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport) - A library using Google Draco to compress the pointclouds.

- [zlib_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/zlib_point_cloud_transport) - A library using zlib to compress the pointclouds.

- [zstd_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/master/zstd_point_cloud_transport) - A library using ZSTD to compress the pointclouds.

More transports can be added. Pull requests are welcome!
