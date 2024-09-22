#!/bin/bash

cd ~/ros2_ws/src/mrover
cd $(pwd)/starter_project/teleop/basestation_gui
python3 manage.py runserver
