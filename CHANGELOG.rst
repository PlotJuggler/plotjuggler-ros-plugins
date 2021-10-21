^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plotjuggler_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.2 (2021-10-21)
------------------
* parse a ROSBAG even if some topic types are not recognized
* Update ros2.yaml
* segmentation fault off (`#30 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/30>`_)
* Contributors: Davide Faconti, simulacrus

1.5.0 (2021-06-20)
------------------
* massive changes
  - include consistent timestamp (suggested by @doisyg )
  - lazy initialization in parsers.
  - reusable Header parser
  - string field added
* add lazy parser inizialization and string field to ROS1
* Contributors: Davide Faconti

1.4.1 (2021-06-18)
------------------
* remove obsolate headers
* forget to install launch file for ROS1
* Contributors: Davide Faconti, Kei Okada

1.3.0 (2021-06-12)
------------------
* use std::any
* temporary fix for 3.2
* Contributors: Davide Faconti

1.2.0 (2021-06-03)
------------------
* fix issue `#15 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/15>`_ for ROS1 too
* Merge pull request `#4 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/4>`_ from Tobias-Fischer/patch-1
  Fix Windows compilation
* fix bugs related to TopicPublishers (ros2)
* Fix isnan issues on Win
* Fix isnan compilation issue on Win
* Fix double-defined ERROR
* Contributors: Davide Faconti, Tobias Fischer

1.1.1 (2021-05-10)
------------------
* Mitigate proble with ros::ok()
* prepare for newer PJ version
* address issue with INT64
* Update README.md
* fix issue `#399 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/399>`_ and `#398 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/398>`_
* Contributors: Davide Faconti

1.1.0 (2021-01-31)
------------------

1.0.3 (2021-01-20)
------------------
* fix bug `#387 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/387>`_ in Plotjuggler repo
* Merge pull request `#3 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/3>`_ from kefrobotics/development
  Add diagnostic_msgs as ROS2 dependency in CMakeLists.txt files
* Add diagnostic_msgs as ROS2 dependency in CMakeLists.txt files
* Contributors: Davide Faconti, Paul Frivold

1.0.2 (2020-12-30)
------------------
* Merge branch 'development' of https://github.com/PlotJuggler/plotjuggler-ros-plugins into development
* fix compilation in ros2
* Contributors: Davide Faconti

1.0.1 (2020-12-18)
------------------
* Added TF messages to the parser (issue `#366 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/366>`_)
* Update README.md
* Update README.md
* Merge pull request `#1 <https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/1>`_ from uhobeike/development
  Made it possible to install
* Made it possible to install
* fix includes
* Contributors: Davide Faconti, davide, uhobeike

1.0.0 (2020-11-23)
------------------

* Initial commit
* Contributors: Davide Faconti
