#!/bin/bash

if [ $1 == 'deep_kinect_tracking' ] || [ $1 == 'deep' ]
then
  source ~/tensorflow/venv/bin/activate
  rosrun popcorn_vision deep_node.py _deep_model:=$2
fi