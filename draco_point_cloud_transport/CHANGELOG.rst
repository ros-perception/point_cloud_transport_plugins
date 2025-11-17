^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package draco_point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.0.3 (2025-11-17)
------------------
* Correctly export dependencies for downstream packages (`#70 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/70>`_)
* Contributors: Albers Franz

5.0.2 (2025-10-15)
------------------
* Fix linking to draco if DRACO_LIBRARIES is not defined (`#66 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/66>`_)
* Contributors: Silvio Traversaro

5.0.1 (2024-09-03)
------------------
* Change Linking to DRACO_LIBRARIES as it was changes in draco (`#57 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/57>`_)
* Use target_link_libraries everywhere (`#52 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/52>`_)
* Contributors: Alejandro Hernández Cordero, Bo Chen

5.0.0 (2024-05-24)
------------------
* Get user specified parameters at startup (`#46 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/46>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Update CI (`#47 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/47>`_)
* Contributors: Alejandro Hernández Cordero, john-maidbot

3.0.3 (2023-02-19)
------------------
* Use tl_expected from rcpputils (`#42 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/42>`_)
* Contributors: Alejandro Hernández Cordero

3.0.2 (2023-12-12)
------------------
* Fixed draco subscriber param names (`#38 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/38>`_)
* Contributors: Alejandro Hernández Cordero

3.0.1 (2023-10-05)
------------------
* Fixed parameter names (`#28 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/28>`_)
* Contributors: Alejandro Hernández Cordero

3.0.0 (2023-09-20)
------------------
* feat: use tl_expected of ros package (`#22 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/22>`_)
* Contributors: Daisuke Nishimatsu

2.0.0 (2023-09-18)
------------------
* Port to ROS 2
* Contributors: Alejandro Hernandez Cordero, john-maidbot

1.0.5 (2023-06-16)
------------------
* Fixed formatting.
* Contributors: Martin Pecka

1.0.4 (2023-06-16)
------------------
* Fixed handling of clouds with invalid points.
* Contributors: Martin Pecka

1.0.3 (2023-05-13)
------------------
* Compatibility with GCC 7.
* Contributors: Martin Pecka

1.0.2 (2023-05-11)
------------------
* Report log messages from the transport via the log helper.
* Hide draco library symbols from include files so that it does not have to be exported.
* Contributors: Martin Pecka

1.0.1 (2023-05-11)
------------------
* More robust Draco CMake finding.
* Adopted direct encoders and decoders.
* Moved draco_point_cloud_transport into its own folder
* Forked from https://github.com/paplhjak/draco_point_cloud_transport
* Contributors: Martin Pecka, Jakub Paplham, Tomas Petricek
