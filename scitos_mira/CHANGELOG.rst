^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_mira
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (26-04-2023)
------------------
* Replace declare_if_not_declared by using nav2_util.
* Added log when terminate the node.
* Reset motorstop if bumper is activated at startup.
* Reset motorstop when emergency button is released.

0.4.0 (25-04-2023)
------------------
* Prepare for humble release.

0.3.1 (12-04-2023)
------------------
* Fill the battery status message.

0.3.0 (10-03-2023)
------------------
* Improve battery status.

0.2.1 (09-12-2022)
------------------
* Replace declare_if_not_declared by using nav2_util.

0.2.0 (07-12-2022)
------------------
* Added visualization of the bumper status.

0.1.5 (10-11-2022)
------------------
* Change to MultiThreadedExecutor.
* Move modules to its own folder.

0.1.4 (28-10-2022)
------------------
* Added header to all messages.

0.1.3 (25-10-2022)
------------------
* Update launch file with arguments.
* Rename folder from config to params.

0.1.2 (13-10-2022)
------------------
* Check if the parameters have been declared.

0.1.1 (11-10-2022)
------------------
* Fix tf broadcaster.

0.1.0 (10-10-2022)
------------------
* Added callback for monitoring parameters.
* Added Display and EBC module.
* Added parameters descriptions.

0.0.2 (07-10-20022)
------------------
* Change casting to static_cast.
* Added base_frame as parameter.
* Added rfid enable service and publish rfid tag.
* Added reset barrier stop service and publish magnet barrier status.
* Improve parameter loading.

0.0.1 (05-10-2022)
------------------
* Create README.md.
* Create CHANGELOG.rst.
* Create CMakeLists.mira.
* Create package.xml.
* Added config file for mira.
* Added launch file for mira.
* Added MIRA log sink.
* Added ModuleFactory.
* Added Charger & Drive modules (ScitosModule)
* Added main ScitosMira node.
* Added CLARC and WeRobot modules.
* Contributors: Alberto Tudela.
