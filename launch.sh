#!/bin/bash
gnome-terminal -e 'roslaunch arm_scenario_simulator baxter_world.launch'
sleep 10
gnome-terminal -e 'rosrun arm_scenario_simulator spawn_objects_example'