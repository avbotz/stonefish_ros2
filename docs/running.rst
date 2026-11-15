=======
Running
=======

Standard graphical simulator
============================

To run the standard simulator you have to execute the ``stonefish_simulator`` node, passing the required command-line arguments:

.. code-block:: console

    $ ros2 run stonefish_ros2 stonefish_simulator <data_path> <description_path> <rate> <window_res_x> <window_res_y> <rendering_quality>

where:

* data_path - the path to the simulation data directory
* description_path - the path to the scenario description file
* rate - the sampling rate of the simulation [Hz]
* widnow_res - the resolution of the main simulator window [px]
* rendering_quality - the quality of the rendering: low, medium or high.

The simulation clock of both simulators is controlled with the following ROS parameters (also exposed as arguments of the provided launch files):

* publish_clock - if ``true``, the simulation time is published on the ``/clock`` topic and used to stamp all messages, so that other nodes can follow it with ``use_sim_time:=true`` (default: ``false``, messages are stamped with the system time)
* realtime_factor - the speed of the simulation relative to real time, e.g. 0.5 runs at half speed (default: 1.0); it can also be changed while the simulation is running, using ``ros2 param set``

The following ROS parameters, set at startup, make the simulation repeatable and independent of the speed of the computer:

* seed - the seed of all random number generators of the simulator, e.g. of the sensor noise; a negative value gives different random numbers in every run (default: -1)
* lock_step - if ``true``, the simulation waits for the renderer and for the publishing of the images, so that every frame of every camera shows the state of the simulation at the time it was due and none is skipped; the simulation then runs slower than real time when the computer cannot keep up (default: ``false``, frames are skipped instead)
* offscreen - if ``true``, the graphical simulator renders without a window and only the cameras are rendered, so that no display is needed, e.g. in automated tests; software rendering can be used by setting ``LIBGL_ALWAYS_SOFTWARE=1`` (default: ``false``)
* start_paused - if ``true``, the simulation starts paused (default: ``false``)

Another option is to include the provided ``launch/stonefish_simulator.launch.py`` file in your own launch file. It is necessary to override the default arguments. See an example below:

.. code-block:: python

    from launch_ros.substitutions import FindPackageShare
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution

    def generate_launch_description():
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('stonefish_ros2'),
                        'launch',
                        'stonefish_simulator.launch.py'
                    ])
                ]),
                launch_arguments = {
                    'simulation_data' : PathJoinSubstitution([FindPackageShare('my_package'), 'data']),
                    'scenario_desc' : PathJoinSubstitution([FindPackageShare('my_package'), 'scenarios', 'simulation.scn']),
                    'simulation_rate' : '300.0',
                    'window_res_x' : '1200',
                    'window_res_y' : '800',
                    'rendering_quality' : 'high'
                }.items()
            )
        ])

Standard console-based simulator (no GPU)
=========================================

To run the standard simulator without the components utilising the GPU and without the graphical window, you have to execute the ``stonefish_simulator_nogpu`` node, passing the required command-line arguments:  

.. code-block:: console

    $ ros2 run stonefish_ros2 stonefish_simulator_nogpu <data_path> <description_path> <rate>

where:

* data_path - the path to the simulation data directory
* description_path - the path to the scenario description file
* rate - the sampling rate of the simulation [Hz]

Another option is to include the provided ``launch/stonefish_simulator_nogpu.launch.py`` file in your own launch file. It is necessary to override the default arguments. See an example below:

.. code-block:: python

    from launch_ros.substitutions import FindPackageShare
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution

    def generate_launch_description():
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('stonefish_ros2'),
                        'launch',
                        'stonefish_simulator_nogpu.launch.py'
                    ])
                ]),
                launch_arguments = {
                    'simulation_data' : PathJoinSubstitution([FindPackageShare('my_package'), 'data']),
                    'scenario_desc' : PathJoinSubstitution([FindPackageShare('my_package'), 'scenarios', 'simulation.scn']),
                    'simulation_rate' : '300.0',
                }.items()
            )
        ])

Implemented services
====================

Some functionality of the simulator node is available through ROS2 services:

* ``pause`` - pause (``true``) or continue (``false``) the simulation (type *std_srvs::srv::SetBool*)

* ``step`` - advance a paused simulation by a number of steps, as fast as possible (type *stonefish_ros2::srv::Step*); the call returns at once, with the simulation time at which the steps will be complete, which can be awaited on the ``/clock`` topic

* ``enable_currents`` - enable simulation of ocean currents (type *std_srvs::srv::Trigger*)

* ``disable_currents`` - disable simulation of ocean currents (type *std_srvs::srv::Trigger*)