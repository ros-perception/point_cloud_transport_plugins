^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package draco_point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.13 (2025-11-17)
-------------------
* Correctly export dependencies for downstream packages (backport `#70 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/70>`_) (`#72 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/72>`_)
* Contributors: mergify[bot]

1.0.12 (2025-10-15)
-------------------
* Fix linking to draco if DRACO_LIBRARIES is not defined (backport `#66 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/66>`_) (`#68 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/68>`_)
* Contributors: mergify[bot]

1.0.11 (2024-05-24)
-------------------
* Get user specified parameters at startup (`#46 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/46>`_) (`#50 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/50>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 46f3d22f3d5660529f4372aeed3fbcb41852a911)
  Co-authored-by: john-maidbot <78750993+john-maidbot@users.noreply.github.com>
* Contributors: mergify[bot]

1.0.10 (2024-02-19)
-------------------
* Use tl_expected from rcpputils (`#42 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/42>`_) (`#44 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/44>`_)
* Contributors: Alejandro Hernández Cordero

1.0.9 (2023-12-12)
------------------
* Fixed draco subscriber param names (`#38 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/38>`_) (`#39 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/39>`_)
  (cherry picked from commit 7d3294d9a3630c989e53b88b16436d6147c88242)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

1.0.8 (2023-10-05)
------------------
* Fixed parameter names (`#28 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/28>`_) (`#29 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/29>`_)
  (cherry picked from commit 45c42b086cadb54ae88a102c6d3802589e267690)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

1.0.7 (2023-09-21)
------------------
* use the right key for draco (`#21 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/21>`_) (`#23 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/23>`_)
  (cherry picked from commit c7b46c442317db84a876cfcafa2dd4b91696d236)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* feat: use tl_expected of ros package (`#22 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/22>`_) (`#25 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/25>`_)
  (cherry picked from commit ad4b632d977a8a06d641bd3fe1b21fc4ba8da0dd)
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
* Contributors: mergify[bot]

1.0.6 (2023-09-20)
------------------
* ROS 2 port.
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
