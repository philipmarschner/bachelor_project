#!/bin/bash

# Call: ./gazebo_client

DIR="$( cd "$( dirname "$1" )" >/dev/null && pwd )"
echo $DIR
export GAZEBO_MODEL_PATH=$DIR/models/:$GAZEBO_MODEL_PATH 

export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/:$GAZEBO_PLUGIN_PATH


gzclient
